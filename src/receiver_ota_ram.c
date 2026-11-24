/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors
*/
/*
 * Receiver self-OTA RAM engine, see receiver_ota_ram.h.
 *
 * Everything executed after takeover lives in the contiguous .ota_ram_engine
 * flash section and is copied into a takeover SRAM arena by the flash-resident
 * launcher rcv_ota_ram_engine_run(). The blob has zero static/global state
 * (all state lives in struct engine_ctx in the arena), uses no library calls,
 * no division, and no tables outside the blob. scripts/check_ota_ram_engine.py
 * verifies this on the linked ELF.
 *
 * Flow (host protocol unchanged, same reports as the staging updater):
 *   READY -> DATA... -> VERIFY (CRC32) -> ACTIVATE -> reset
 * Page 0 stays in RAM until ACTIVATE. It is validated (vector table, MCUboot
 * magic) and erased before any other page is modified. At ACTIVATE bank 0
 * is marked invalid in the bootloader settings page, page 0 is written, and
 * bank 0 is then marked valid with the new size and CRC16, so an interrupted
 * transfer or activation leaves the receiver in UF2 mode.
 */
#include "receiver_ota_ram.h"
#include "esb_ota.h"
#include "receiver_ota.h"

#include <stddef.h>
#include <zephyr/toolchain.h>
#include <nrfx.h>

#define PAGE_SIZE             4096u
#define PAGE_MASK             (PAGE_SIZE - 1u)
#define PAGE_WORDS            (PAGE_SIZE / 4u)
#define REPORT_SIZE           64u
#define DATA_PAYLOAD          60u
#define MIN_IMAGE_SIZE        0x20u       /* Minimum vector table size (8 words) */
#define MCUBOOT_IMAGE_MAGIC   0x96F3B83Du
#define UF2_RECOVERY_MAGIC    0x57u       /* Adafruit UF2 bootloader DFU magic */
#define BL_BANK_VALID_APP     0x0001u     /* Adafruit bootloader settings: bank_0 */
#define BL_BANK_INVALID_APP   0x00FFu     /* Adafruit bootloader settings: invalid bank */
#define CYCLES_PER_MS         64000u      /* nRF52 HCLK (64 MHz) */
#define MS(x)                 ((uint32_t)(x) * CYCLES_PER_MS)
#define IDLE_TIMEOUT          MS(15000)
#define ENGINE_MAX_BLOB_SIZE  4096u
#define ENGINE_STACK_SIZE     2048u
#define ENGINE_SP_GUARD       1024u

/* nRF52 USBD anomaly 199: no tasks during EasyDMA unless this is set. */
#define USBD_ERRATA_199_REG (*(volatile uint32_t *)0x40027C1Cu)

#define OTA_RAM_ENTRY __attribute__((section(".ota_ram_engine.entry"), used, noreturn, noinline))
#define OTA_RAM_TEXT  __attribute__((section(".ota_ram_engine.text"), noinline))

extern const uint8_t __ota_ram_engine_start[];
extern const uint8_t __ota_ram_engine_end[];

struct engine_ctx {
	uint32_t image_size;
	uint32_t image_crc32;
	uint32_t base;
	uint32_t flash_end;
	uint32_t sram_base;
	uint32_t sram_end;
	uint32_t settings_addr;
	uint8_t in_ep;
	uint8_t out_ep;
	uint8_t errata_199;
	uint8_t status;
	uint8_t flash_touched;
	uint8_t verified;
	uint8_t in_pending;
	uint8_t failed;
	uint16_t next_seq;
	uint32_t bytes;
	uint32_t last_rx;
	uint8_t *page0;
	uint8_t *page;
	uint8_t rx_buf[REPORT_SIZE] __aligned(4);
	uint8_t tx_buf[REPORT_SIZE] __aligned(4);
};

/* ── Timing, WDT, Reset (all inside .ota_ram_engine.text) ────────── */

static inline __attribute__((always_inline)) uint32_t cycles(void)
{
	return DWT->CYCCNT;
}

static OTA_RAM_TEXT void wdt_feed(void)
{
	if (NRF_WDT->RUNSTATUS & WDT_RUNSTATUS_RUNSTATUS_Msk) {
		uint32_t rren = NRF_WDT->RREN;

		for (uint32_t i = 0; i < 8u; i++) {
			if (rren & (1u << i)) {
				NRF_WDT->RR[i] = WDT_RR_RR_Reload;
			}
		}
	}
}

static OTA_RAM_TEXT void delay_cycles(uint32_t n)
{
	uint32_t t0 = cycles();

	while ((uint32_t)(cycles() - t0) < n) {
		wdt_feed();
	}
}

static OTA_RAM_TEXT __attribute__((noreturn)) void engine_reset(struct engine_ctx *ctx)
{
	if (ctx->status == OTA_STATUS_COMPLETE) {
		NRF_POWER->GPREGRET = 0u;
	} else if (ctx->flash_touched) {
		/* Page 0 is erased or partially written: force UF2 recovery mode. */
		NRF_POWER->GPREGRET = UF2_RECOVERY_MAGIC;
	}
	__DSB();
	SCB->AIRCR = (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
		     (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) | SCB_AIRCR_SYSRESETREQ_Msk;
	__DSB();
	for (;;) {
	}
}

/* ── Flash ───────────────────────────────────────────────────────── */

static OTA_RAM_TEXT void nvmc_wait(void)
{
	while (NRF_NVMC->READY == 0u) {
		wdt_feed();
	}
}

static OTA_RAM_TEXT bool in_app(const struct engine_ctx *ctx, uint32_t addr)
{
	return addr >= ctx->base && addr < ctx->flash_end &&
	       (addr & PAGE_MASK) == 0u && ctx->flash_end - addr >= PAGE_SIZE;
}

/* Callers check the address: an application page or the settings page. */
static OTA_RAM_TEXT bool nvmc_erase(struct engine_ctx *ctx, uint32_t addr)
{

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
	__DSB();
	nvmc_wait();
	NRF_NVMC->ERASEPAGE = addr;
	__DSB();
	nvmc_wait();
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	__DSB();
	__ISB();
	nvmc_wait();
	ctx->flash_touched = 1u;

	const volatile uint32_t *dst = (const volatile uint32_t *)addr;

	for (uint32_t i = 0; i < PAGE_WORDS; i++) {
		if (dst[i] != 0xFFFFFFFFu) {
			return false;
		}
	}
	return true;
}

static OTA_RAM_TEXT bool nvmc_program(struct engine_ctx *ctx, uint32_t addr,
				      const uint8_t *src)
{
	const uint32_t *words = (const uint32_t *)src;
	volatile uint32_t *dst = (volatile uint32_t *)addr;

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
	__DSB();
	nvmc_wait();
	/* Descending, so word 0 lands last: the settings bank marker becomes
	 * valid only after the size, and page 0's vector table after its body. */
	for (uint32_t i = PAGE_WORDS; i-- > 0u;) {
		if (words[i] != 0xFFFFFFFFu) {
			dst[i] = words[i];
			__DSB();
			nvmc_wait();
		}
	}
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	__DSB();
	__ISB();
	nvmc_wait();
	ctx->flash_touched = 1u;

	for (uint32_t i = 0; i < PAGE_WORDS; i++) {
		if (dst[i] != words[i]) {
			return false;
		}
	}
	return true;
}

static OTA_RAM_TEXT bool flash_erase(struct engine_ctx *ctx, uint32_t addr)
{
	return in_app(ctx, addr) && nvmc_erase(ctx, addr);
}

static OTA_RAM_TEXT bool flash_program_page(struct engine_ctx *ctx, uint32_t addr,
					    const uint8_t *src)
{
	return in_app(ctx, addr) && nvmc_program(ctx, addr, src);
}

/* ── USB (bare USBD, endpoints already configured by Zephyr) ─────── */

static OTA_RAM_TEXT void dma_begin(const struct engine_ctx *ctx)
{
	if (ctx->errata_199) {
		USBD_ERRATA_199_REG = 0x00000082u;
		__DSB();
	}
}

static OTA_RAM_TEXT void dma_end(const struct engine_ctx *ctx)
{
	if (ctx->errata_199) {
		USBD_ERRATA_199_REG = 0x00000000u;
		__DSB();
	}
}

static OTA_RAM_TEXT bool wait_event(volatile uint32_t *event, uint32_t timeout)
{
	uint32_t t0 = cycles();

	while (*event == 0u) {
		wdt_feed();
		if ((uint32_t)(cycles() - t0) > timeout) {
			return false;
		}
	}
	return true;
}

static OTA_RAM_TEXT void wait_in_consumed(struct engine_ctx *ctx, uint32_t timeout)
{
	uint32_t ep_mask = 1u << ctx->in_ep;
	uint32_t t0 = cycles();

	while (ctx->in_pending && (uint32_t)(cycles() - t0) < timeout) {
		wdt_feed();
		if (NRF_USBD->EVENTS_USBRESET) {
			engine_reset(ctx);
		}
		if (NRF_USBD->EPDATASTATUS & ep_mask) {
			NRF_USBD->EPDATASTATUS = ep_mask;
			ctx->in_pending = 0u;
			break;
		}
	}
}

static OTA_RAM_TEXT void send_status(struct engine_ctx *ctx, uint8_t code)
{
	volatile uint8_t *tx = ctx->tx_buf;
	uint32_t ep = ctx->in_ep;

	/* Let the host take the previous report first. Only the IN bit is
	 * cleared here, so pending OUT data is still seen by usb_poll(). */
	wait_in_consumed(ctx, MS(20));

	for (uint32_t i = 0; i < REPORT_SIZE; i++) {
		tx[i] = 0u;
	}
	tx[0] = HID_OTA_STATUS;
	tx[1] = RECEIVER_OTA_ID;
	tx[2] = code;
	tx[3] = (uint8_t)(ctx->next_seq >> 8);
	tx[4] = (uint8_t)ctx->next_seq;
	tx[5] = (uint8_t)ctx->bytes;
	tx[6] = (uint8_t)(ctx->bytes >> 8);
	tx[7] = (uint8_t)(ctx->bytes >> 16);
	tx[8] = (uint8_t)(ctx->bytes >> 24);
	__DMB();

	/* Clear any stale IN completion bit before arming the new packet. */
	NRF_USBD->EPDATASTATUS = 1u << ep;
	NRF_USBD->EVENTS_ENDEPIN[ep] = 0u;
	dma_begin(ctx);
	NRF_USBD->EPIN[ep].PTR = (uint32_t)ctx->tx_buf;
	NRF_USBD->EPIN[ep].MAXCNT = REPORT_SIZE;
	NRF_USBD->TASKS_STARTEPIN[ep] = 1u;
	bool done = wait_event(&NRF_USBD->EVENTS_ENDEPIN[ep], MS(5));

	NRF_USBD->EVENTS_ENDEPIN[ep] = 0u;
	dma_end(ctx);
	ctx->in_pending = done ? 1u : 0u;
}

static OTA_RAM_TEXT void fail(struct engine_ctx *ctx, uint8_t code)
{
	ctx->failed = 1u;
	ctx->status = code;
	send_status(ctx, code);
}

/* ── Checksums ───────────────────────────────────────────────────── */

static OTA_RAM_TEXT uint32_t crc32_update(uint32_t crc, const volatile uint8_t *src, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++) {
		if ((i & 0x3FFu) == 0u) {
			wdt_feed();
		}
		crc ^= src[i];
		for (uint32_t b = 0; b < 8u; b++) {
			crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
		}
	}
	return crc;
}

static OTA_RAM_TEXT uint32_t crc32_image(const struct engine_ctx *ctx)
{
	uint32_t crc = 0xFFFFFFFFu;
	uint32_t p0_len = ctx->image_size < PAGE_SIZE ? ctx->image_size : PAGE_SIZE;

	crc = crc32_update(crc, ctx->page0, p0_len);
	if (ctx->image_size > PAGE_SIZE) {
		crc = crc32_update(crc, (const volatile uint8_t *)(ctx->base + PAGE_SIZE),
				   ctx->image_size - PAGE_SIZE);
	}
	return ~crc;
}

/* Nordic SDK crc16_compute step, as used by the Adafruit bootloader. */
static OTA_RAM_TEXT uint16_t crc16_step(uint16_t crc, uint8_t byte)
{
	crc = (uint8_t)(crc >> 8) | (uint16_t)(crc << 8);
	crc ^= byte;
	crc ^= (uint8_t)(crc & 0xFFu) >> 4;
	crc ^= (uint16_t)((crc << 8) << 4);
	crc ^= (uint16_t)(((crc & 0xFFu) << 4) << 1);
	return crc;
}

static OTA_RAM_TEXT uint16_t crc16_update(uint16_t crc, const volatile uint8_t *src, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++) {
		if ((i & 0x3FFu) == 0u) {
			wdt_feed();
		}
		crc = crc16_step(crc, src[i]);
	}
	return crc;
}

static OTA_RAM_TEXT uint16_t crc16_image(const struct engine_ctx *ctx, uint32_t end)
{
	uint16_t crc = 0xFFFFu;
	uint32_t p0_len = end < PAGE_SIZE ? end : PAGE_SIZE;

	crc = crc16_update(crc, ctx->page0, p0_len);
	if (end > PAGE_SIZE) {
		crc = crc16_update(crc, (const volatile uint8_t *)(ctx->base + PAGE_SIZE),
				   end - PAGE_SIZE);
	}
	return crc;
}

/* ── Protocol ────────────────────────────────────────────────────── */

static OTA_RAM_TEXT void pad_page(uint8_t *buf, uint32_t from)
{
	volatile uint8_t *dst = buf;

	for (uint32_t i = from; i < PAGE_SIZE; i++) {
		dst[i] = 0xFFu;
	}
}

static inline __attribute__((always_inline)) uint32_t load_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Validate page 0 before erasing anything so bad images keep the old app intact. */
static OTA_RAM_TEXT uint8_t validate_page0(const struct engine_ctx *ctx)
{
	uint32_t init_sp = load_le32(ctx->page0);
	uint32_t reset_vec = load_le32(ctx->page0 + 4u);

	if (init_sp == MCUBOOT_IMAGE_MAGIC) {
		return OTA_STATUS_VERIFY_FAIL;
	}
	if ((init_sp & 3u) != 0u || init_sp <= ctx->sram_base || init_sp > ctx->sram_end) {
		return OTA_STATUS_VERIFY_FAIL;
	}
	if ((reset_vec & 1u) == 0u ||
	    reset_vec < ctx->base || reset_vec >= ctx->base + ctx->image_size) {
		return OTA_STATUS_VERIFY_FAIL;
	}
	return 0u;
}

/*
 * Rewrite bank 0 of the bootloader settings page, keeping the rest of the
 * page. With valid=false bank 0 is marked invalid, so the bootloader stays in
 * UF2 mode whatever state page 0 is in. With valid=true it gets the new
 * image's CRC16 and size.
 */
static OTA_RAM_TEXT bool write_settings(struct engine_ctx *ctx, bool valid)
{
	const uint32_t addr = ctx->settings_addr;
	const volatile uint32_t *cur = (const volatile uint32_t *)addr;
	volatile uint32_t *buf = (volatile uint32_t *)ctx->page; /* no memcpy call */
	uint16_t crc = crc16_image(ctx, ctx->image_size);

	for (uint32_t i = 0; i < PAGE_WORDS; i++) {
		buf[i] = cur[i];
	}
	buf[0] = (valid ? BL_BANK_VALID_APP : BL_BANK_INVALID_APP) |
		 ((uint32_t)crc << 16);                         /* bank_0, bank_0_crc */
	buf[1] = (buf[1] & 0xFFFF0000u) | BL_BANK_INVALID_APP;  /* bank_1, padding */
	buf[2] = ctx->image_size;                                /* bank_0_size */
	return nvmc_erase(ctx, addr) && nvmc_program(ctx, addr, ctx->page);
}

/* Called when page `index` is complete (or the image ends inside it). */
static OTA_RAM_TEXT void page_done(struct engine_ctx *ctx, uint32_t index)
{
	uint32_t fill = ctx->bytes - index * PAGE_SIZE;

	if (index == 0u) {
		pad_page(ctx->page0, fill);
		uint8_t err = validate_page0(ctx);

		if (err != 0u) {
			/* Nothing written yet: the old application is intact. */
			fail(ctx, err);
			return;
		}
		/* Invalidate the running image before overwriting any of it. */
		if (!flash_erase(ctx, ctx->base)) {
			fail(ctx, OTA_STATUS_FLASH_ERROR);
			return;
		}
		ctx->status = OTA_STATUS_RECEIVING;
		send_status(ctx, OTA_STATUS_RECEIVING);
		return;
	}

	uint32_t addr = ctx->base + index * PAGE_SIZE;

	if (addr < ctx->base || addr >= ctx->flash_end || ctx->flash_end - addr < PAGE_SIZE) {
		fail(ctx, OTA_STATUS_FLASH_ERROR);
		return;
	}

	pad_page(ctx->page, fill);

	/* Skip erase/write when the destination page already matches word-for-word. */
	const uint32_t *src = (const uint32_t *)ctx->page;
	const volatile uint32_t *dst = (const volatile uint32_t *)addr;
	bool same = true;

	for (uint32_t i = 0; i < PAGE_WORDS; i++) {
		if (dst[i] != src[i]) {
			same = false;
			break;
		}
	}
	if (!same) {
		if (!flash_erase(ctx, addr) || !flash_program_page(ctx, addr, ctx->page)) {
			fail(ctx, OTA_STATUS_FLASH_ERROR);
			return;
		}
	}
	ctx->status = OTA_STATUS_RECEIVING;
	send_status(ctx, OTA_STATUS_RECEIVING);
}

static OTA_RAM_TEXT void handle_data(struct engine_ctx *ctx, const uint8_t *pkt, uint32_t len)
{
	if (ctx->failed || ctx->verified || len < 4u || ctx->bytes >= ctx->image_size) {
		return;
	}

	uint16_t seq = (uint16_t)(((uint16_t)pkt[2] << 8) | pkt[3]);

	if (seq != ctx->next_seq) {
		if ((uint16_t)(seq - ctx->next_seq) < 0x8000u) {
			fail(ctx, OTA_STATUS_SEQ_ERROR);
		}
		return; /* Older sequence: duplicate */
	}

	uint32_t n = ctx->image_size - ctx->bytes;

	if (n > DATA_PAYLOAD) {
		n = DATA_PAYLOAD;
	}
	/* Reject short chunks rather than shifting subsequent image offsets. */
	if (len - 4u < n) {
		return;
	}

	for (uint32_t i = 0; i < n && !ctx->failed; i++) {
		uint32_t off = ctx->bytes;
		uint8_t *buf = off < PAGE_SIZE ? ctx->page0 : ctx->page;

		buf[off & PAGE_MASK] = pkt[4u + i];
		ctx->bytes = off + 1u;
		if ((ctx->bytes & PAGE_MASK) == 0u || ctx->bytes == ctx->image_size) {
			page_done(ctx, off >> 12);
		}
	}
	if (!ctx->failed) {
		ctx->next_seq++;
	}
}

static OTA_RAM_TEXT void handle_verify(struct engine_ctx *ctx)
{
	if (ctx->failed) {
		send_status(ctx, ctx->status);
		return;
	}
	if (ctx->verified) {
		send_status(ctx, OTA_STATUS_VERIFY_OK);
		return;
	}
	if (ctx->bytes != ctx->image_size) {
		fail(ctx, OTA_STATUS_SEQ_ERROR);
		return;
	}
	if (crc32_image(ctx) != ctx->image_crc32) {
		fail(ctx, OTA_STATUS_VERIFY_FAIL);
		return;
	}
	ctx->verified = 1u;
	ctx->status = OTA_STATUS_VERIFY_OK;
	send_status(ctx, OTA_STATUS_VERIFY_OK);
}

static OTA_RAM_TEXT void handle_activate(struct engine_ctx *ctx)
{
	if (!ctx->verified) {
		send_status(ctx, ctx->failed ? ctx->status : OTA_STATUS_ERROR);
		return;
	}
	ctx->status = OTA_STATUS_ACTIVATING;
	send_status(ctx, OTA_STATUS_ACTIVATING);

	/* Bank 0 stays invalid while page 0 is programmed, so a power loss
	 * here cannot boot a partly written page 0. A loss while the final
	 * settings write erases the page only exposes the complete image. */
	if (ctx->settings_addr != 0u && !write_settings(ctx, false)) {
		fail(ctx, OTA_STATUS_FLASH_ERROR);
		return;
	}
	if (!flash_program_page(ctx, ctx->base, ctx->page0) ||
	    (ctx->settings_addr != 0u && !write_settings(ctx, true))) {
		fail(ctx, OTA_STATUS_FLASH_ERROR);
		return;
	}
	ctx->status = OTA_STATUS_COMPLETE;
	send_status(ctx, OTA_STATUS_COMPLETE);
	wait_in_consumed(ctx, MS(50));
	delay_cycles(MS(10));
	engine_reset(ctx);
}

static OTA_RAM_TEXT void handle_report(struct engine_ctx *ctx, const uint8_t *pkt, uint32_t len)
{
	if (len < 2u) {
		return;
	}

	uint8_t type = pkt[0];
	uint8_t target = pkt[1];

	/* Accept broadcast ABORT (0xFF) as sent by esb_ota.py on Ctrl-C. */
	if (type == HID_OTA_ABORT && (target == RECEIVER_OTA_ID || target == 0xFFu)) {
		engine_reset(ctx);
	}
	if (target != RECEIVER_OTA_ID) {
		return;
	}
	ctx->last_rx = cycles();

	if (type == HID_OTA_DATA) {
		handle_data(ctx, pkt, len);
	} else if (type == HID_OTA_VERIFY) {
		handle_verify(ctx);
	} else if (type == HID_OTA_ACTIVATE) {
		handle_activate(ctx);
	} else if (type == HID_OTA_BEGIN) {
		send_status(ctx, ctx->status);
	}
}

static OTA_RAM_TEXT void receive(struct engine_ctx *ctx)
{
	uint32_t ep = ctx->out_ep;
	uint32_t size = NRF_USBD->SIZE.EPOUT[ep];

	if (size == 0u) {
		NRF_USBD->SIZE.EPOUT[ep] = 0u;
		return;
	}
	if (size > REPORT_SIZE) {
		size = REPORT_SIZE;
	}

	NRF_USBD->EVENTS_ENDEPOUT[ep] = 0u;
	dma_begin(ctx);
	NRF_USBD->EPOUT[ep].PTR = (uint32_t)ctx->rx_buf;
	NRF_USBD->EPOUT[ep].MAXCNT = size;
	NRF_USBD->TASKS_STARTEPOUT[ep] = 1u;
	bool done = wait_event(&NRF_USBD->EVENTS_ENDEPOUT[ep], MS(5));

	NRF_USBD->EVENTS_ENDEPOUT[ep] = 0u;
	dma_end(ctx);
	__DMB();
	__asm volatile("" ::: "memory");

	if (done) {
		handle_report(ctx, ctx->rx_buf, NRF_USBD->EPOUT[ep].AMOUNT);
	} else {
		NRF_USBD->SIZE.EPOUT[ep] = 0u;
	}
}

static OTA_RAM_TEXT void usb_poll(struct engine_ctx *ctx)
{
	if (NRF_USBD->EVENTS_USBRESET) {
		/* Host reset the bus: the session cannot continue. */
		engine_reset(ctx);
	}
	if (NRF_USBD->EVENTS_EP0SETUP) {
		/* No control requests are served while flashing. */
		NRF_USBD->EVENTS_EP0SETUP = 0u;
		NRF_USBD->TASKS_EP0STALL = 1u;
	}
	if (NRF_USBD->EVENTS_USBEVENT) {
		NRF_USBD->EVENTS_USBEVENT = 0u;
		NRF_USBD->EVENTCAUSE = NRF_USBD->EVENTCAUSE;
		NRF_USBD->LOWPOWER = 0u;
	}

	NRF_USBD->EVENTS_EPDATA = 0u;
	uint32_t st = NRF_USBD->EPDATASTATUS;

	if (st == 0u) {
		return;
	}
	NRF_USBD->EPDATASTATUS = st;
	if (st & (1u << ctx->in_ep)) {
		ctx->in_pending = 0u;
	}
	if (st & (1u << (16u + ctx->out_ep))) {
		receive(ctx);
	}
}

/* Entry point pinned at offset 0 of .ota_ram_engine. Runs 100% from RAM. */
static OTA_RAM_ENTRY void ota_ram_engine_entry(struct engine_ctx *ctx)
{
	if (NRF_USBD->EVENTS_USBRESET) {
		engine_reset(ctx);
	}

	/* Allow any Zephyr-queued HID IN packet to drain before replacing it. */
	uint32_t in_mask = 1u << ctx->in_ep;
	uint32_t t0 = cycles();

	while ((uint32_t)(cycles() - t0) < MS(5)) {
		wdt_feed();
		if (NRF_USBD->EVENTS_USBRESET) {
			engine_reset(ctx);
		}
		if (NRF_USBD->EPDATASTATUS & in_mask) {
			NRF_USBD->EPDATASTATUS = in_mask;
			break;
		}
	}

	/* Drop any residual OUT packet; the host sends no DATA before READY. */
	NRF_USBD->SIZE.EPOUT[ctx->out_ep] = 0u;
	NRF_USBD->EVENTS_EPDATA = 0u;
	NRF_USBD->EPDATASTATUS = NRF_USBD->EPDATASTATUS;
	NRF_USBD->EVENTS_USBEVENT = 0u;
	NRF_USBD->EVENTCAUSE = NRF_USBD->EVENTCAUSE;
	NRF_USBD->LOWPOWER = 0u;

	ctx->status = OTA_STATUS_READY;
	ctx->last_rx = cycles();
	send_status(ctx, OTA_STATUS_READY);

	for (;;) {
		wdt_feed();
		usb_poll(ctx);
		if ((uint32_t)(cycles() - ctx->last_rx) > IDLE_TIMEOUT) {
			engine_reset(ctx);
		}
	}
}

/* ── Flash-resident launcher (runs before flash is touched) ──────── */

bool rcv_ota_ram_engine_run(const struct rcv_ota_ram_params *p)
{
	if (p == NULL) {
		return false;
	}

	uintptr_t blob_start = (uintptr_t)__ota_ram_engine_start;
	uintptr_t blob_end = (uintptr_t)__ota_ram_engine_end;
	uint32_t blob_size = (uint32_t)(blob_end - blob_start);

	if (blob_end <= blob_start || (blob_start & 3u) != 0u || (blob_size & 3u) != 0u ||
	    blob_size > ENGINE_MAX_BLOB_SIZE) {
		return false;
	}
	if (p->flash_base < 0x1000u || (p->flash_base & PAGE_MASK) != 0u ||
	    (p->flash_end & PAGE_MASK) != 0u || p->flash_end <= p->flash_base) {
		return false;
	}
	if (p->image_size < MIN_IMAGE_SIZE || p->image_size > p->flash_end - p->flash_base) {
		return false;
	}
	if (p->settings_addr != 0u &&
	    ((p->settings_addr & PAGE_MASK) != 0u || p->settings_addr < p->flash_end)) {
		return false;
	}
	if (p->in_ep == 0u || p->in_ep > 7u || p->out_ep == 0u || p->out_ep > 7u) {
		return false;
	}

	uint32_t ctx_size = (uint32_t)((sizeof(struct engine_ctx) + 7u) & ~7u);
	uint32_t code_size = (blob_size + 7u) & ~7u;
	uint32_t arena_size = (2u * PAGE_SIZE) + ctx_size + code_size + ENGINE_STACK_SIZE;

	if (p->sram_end <= p->sram_base || (p->sram_base & 7u) != 0u ||
	    p->sram_end - p->sram_base < (2u * arena_size) + ENGINE_SP_GUARD) {
		return false;
	}

	/* Snapshot parameters on the current stack before taking over SRAM. */
	struct rcv_ota_ram_params lp = *p;

	/* Point of no return: mask interrupts and quiesce all EasyDMA producers
	 * while still executing from intact flash, before touching the SRAM arena. */
	__asm volatile("cpsid i" ::: "memory");
	SysTick->CTRL = 0u;
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0u;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	NRF_PPI->CHENCLR = 0xFFFFFFFFu;
	NRF_RADIO->TASKS_DISABLE = 1u;
#ifdef NRF_PWM0
	NRF_PWM0->TASKS_STOP = 1u;
#endif
#ifdef NRF_PWM1
	NRF_PWM1->TASKS_STOP = 1u;
#endif
#ifdef NRF_SPIM0
	NRF_SPIM0->TASKS_STOP = 1u;
#endif
#ifdef NRF_SPIM1
	NRF_SPIM1->TASKS_STOP = 1u;
#endif
#ifdef NRF_SPIM2
	NRF_SPIM2->TASKS_STOP = 1u;
#endif
#ifdef NRF_SPIM3
	NRF_SPIM3->TASKS_STOP = 1u;
#endif

	uint32_t t0 = DWT->CYCCNT;

	while ((uint32_t)(DWT->CYCCNT - t0) < MS(2)) {
		if (NRF_WDT->RUNSTATUS & WDT_RUNSTATUS_RUNSTATUS_Msk) {
			NRF_WDT->RR[0] = WDT_RR_RR_Reload;
		}
	}

	/* Clear any Errata 199 flag left active by an interrupted Zephyr DMA. */
	if (lp.errata_199) {
		USBD_ERRATA_199_REG = 0x00000000u;
		__DSB();
	}
	NRF_USBD->INTEN = 0u;
	NRF_USBD->LOWPOWER = 0u;

	/* Disable MPU so the copied code in SRAM is executable (SRAM is XN under Zephyr). */
	MPU->CTRL = 0u;
	__DSB();
	__ISB();

	/* Place the takeover arena in SRAM disjoint from the active stack frame. */
	uintptr_t sp;

	__asm volatile("mov %0, sp" : "=r"(sp));
	uintptr_t arena = (sp >= (uintptr_t)lp.sram_base + arena_size + ENGINE_SP_GUARD)
				  ? (uintptr_t)lp.sram_base
				  : (((uintptr_t)lp.sram_end - arena_size) & ~(uintptr_t)7u);

	uint8_t *page0 = (uint8_t *)arena;
	uint8_t *page = (uint8_t *)(arena + PAGE_SIZE);
	struct engine_ctx *ctx = (struct engine_ctx *)(arena + (2u * PAGE_SIZE));
	uintptr_t code_dst = arena + (2u * PAGE_SIZE) + ctx_size;
	uintptr_t stack_top = (arena + arena_size) & ~(uintptr_t)7u;

	/* Zero ctx and copy the position-independent engine blob from flash to RAM. */
	volatile uint32_t *ctx_w = (volatile uint32_t *)ctx;

	for (uint32_t i = 0; i < ctx_size / 4u; i++) {
		ctx_w[i] = 0u;
	}

	const uint32_t *src_w = (const uint32_t *)blob_start;
	volatile uint32_t *dst_w = (volatile uint32_t *)code_dst;

	for (uint32_t i = 0; i < blob_size / 4u; i++) {
		dst_w[i] = src_w[i];
	}

	ctx->image_size = lp.image_size;
	ctx->image_crc32 = lp.image_crc32;
	ctx->base = lp.flash_base;
	ctx->flash_end = lp.flash_end;
	ctx->sram_base = lp.sram_base;
	ctx->sram_end = lp.sram_end;
	ctx->settings_addr = lp.settings_addr;
	ctx->in_ep = lp.in_ep;
	ctx->out_ep = lp.out_ep;
	ctx->errata_199 = lp.errata_199 ? 1u : 0u;
	ctx->page0 = page0;
	ctx->page = page;

	__DSB();
	__ISB();

	/* Switch to the arena's stack and enter the RAM copy (Thumb bit set). */
	register uintptr_t r_ctx __asm("r0") = (uintptr_t)ctx;
	register uintptr_t r_sp __asm("r1") = stack_top;
	register uintptr_t r_pc __asm("r2") = code_dst | 1u;

	__asm volatile(
		"mov sp, %1\n\t"
		"bx  %2\n\t"
		:
		: "r"(r_ctx), "r"(r_sp), "r"(r_pc)
		: "memory"
	);
	__builtin_unreachable();
}
