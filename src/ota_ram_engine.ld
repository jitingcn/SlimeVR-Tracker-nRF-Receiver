/*
 * Position-independent receiver self-OTA RAM engine blob.
 * Stored in flash (ROMABLE_REGION) and copied into a takeover SRAM arena
 * only when a self-OTA session begins, avoiding any permanent .ramfunc cost.
 */
SECTION_PROLOGUE(.ota_ram_engine,,)
{
	. = ALIGN(4);
	__ota_ram_engine_start = .;
	KEEP(*(.ota_ram_engine.entry))
	*(.ota_ram_engine.text)
	*(.ota_ram_engine.text.*)
	. = ALIGN(4);
	__ota_ram_engine_end = .;
} GROUP_LINK_IN(ROMABLE_REGION)
