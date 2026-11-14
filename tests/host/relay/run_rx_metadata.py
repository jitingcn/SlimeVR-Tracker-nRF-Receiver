"""Run actual SDK FIFO push/pop with per-entry metadata and delayed drains."""
from pathlib import Path
import sys, os, subprocess, tempfile, shlex
here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent/'harness'))
from c_extract import extract_block
sdk=here.parents[3]/'nrf'
s=(sdk/'subsys/esb/esb.c').read_text()
body='\n'.join(extract_block(s,rf'^(?:static bool|int) {n}\([^;{{]*\)\s*\{{') for n in ('rx_fifo_push_rfbuf','esb_read_rx_payload'))
h=(sdk/'include/esb.h').read_text(); payload=h[h.index('struct esb_payload {'):h.index('/** @brief Enhanced ShockBurst event.')]
prefix=r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#define CONFIG_ESB_MAX_PAYLOAD_LENGTH 72
#define CONFIG_ESB_RX_FIFO_SIZE 3
#define ESB_PROTOCOL_ESB_DPL 1
#define ESB_MODE_PTX 2
#define ESB_STATE_UNINITIALIZED 0
#define NRF_RADIO 0
static unsigned ticks, generation=1;
static unsigned k_uptime_ticks(void){return ticks;}
static int atomic_get(int *p){return *p;}
static void atomic_inc(int *p){++*p;}
static void atomic_dec(int *p){--*p;}
static int nrf_radio_rssi_sample_get(int p){(void)p;return -50;}
static struct {int protocol,mode,payload_length;} esb_cfg={1,1,17};
static int esb_state=1,rx_fifo_full,rx_invalid_payload_length;
struct esb_radio_pdu {struct {struct {uint8_t length,ack;} dpl_pdu;} type; uint8_t data[72];};
static struct esb_radio_pdu buffer;
#define rx_payload_buffer (&buffer)
'''
prefix+=payload+r'''
static struct esb_payload entries[3];
static struct {int count; unsigned front,back;struct esb_payload *payload[3];} rx_fifo={.payload={entries,entries+1,entries+2}};
static void metadata(struct esb_payload *p){p->rx_context=generation;p->rx_pose=p->data[0]==17;}
static void (*rx_metadata_handler)(struct esb_payload *)=metadata;
'''
cases=r'''
int main(void){
 buffer.type.dpl_pdu.length=17;buffer.data[1]=2;
 ticks=100;buffer.data[0]=17;assert(rx_fifo_push_rfbuf(3,0,k_uptime_ticks()));
 ticks=110;buffer.data[0]=0;assert(rx_fifo_push_rfbuf(3,1,k_uptime_ticks()));
 ticks=120;buffer.data[0]=17;assert(rx_fifo_push_rfbuf(3,2,k_uptime_ticks()));
 assert(!rx_fifo_push_rfbuf(3,3,k_uptime_ticks()));
 ticks=1000;generation++;
 struct esb_payload p;
 assert(!esb_read_rx_payload(&p)&&p.rx_ticks==100&&p.rx_pose&&p.rx_context==1);
 assert(!esb_read_rx_payload(&p)&&p.rx_ticks==110&&!p.rx_pose);
 assert(!esb_read_rx_payload(&p)&&p.rx_ticks==120&&p.rx_pose);
 assert(esb_read_rx_payload(&p)==-ENODATA);
 ticks=1300;assert(rx_fifo_push_rfbuf(3,0,k_uptime_ticks()));assert(!esb_read_rx_payload(&p)&&p.rx_context==2);
 return 0;}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t);(p/'test.c').write_text(prefix+body+cases)
 subprocess.run(shlex.split(os.getenv('CC','cc'))+['-std=c11','-Wall','-Werror','-fsanitize=address,undefined',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('FIFO receive timestamp/classification/generation survive delayed drains, full queue and wrap')
