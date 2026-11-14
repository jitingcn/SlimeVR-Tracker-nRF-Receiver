"""Exercise actual bounded drop injection and legacy finally restoration."""
from pathlib import Path as _RelayPath
import sys as _relay_sys
_relay_sys.path.insert(0, str(_RelayPath(__file__).resolve().parents[1] / 'harness'))
from relay_config import select_relay

import os, shlex, subprocess, sys, tempfile
from pathlib import Path
here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[3]/'tracker/tests/host/harness/python'))
from c_extract import extract_block
src=select_relay((here.parents[2]/'src/connection/esb.c').read_text())
code='''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "relay_protocol.h"
#define RC_MAX 16
#define BIT(i) (1U << (i))
#define MIN(a,b) ((a)<(b)?(a):(b))
static uint8_t relay_block_id=255, relay_block_percent;
static uint16_t relay_block_mask;
static uint32_t relay_block_deadline, now;
static struct {unsigned blocked;} relay_rx_stats;
static unsigned irq_lock(void){return 0;}
static void irq_unlock(unsigned k){(void)k;}
static uint32_t k_uptime_get_32(void){return now;}
'''
for name in ('relay_multi_blocked','relay_block_drop','esb_relay_block','esb_relay_block_mask'):
 code+='\n'+extract_block(src,rf'^(?:static )?(?:bool|void) {name}\([^;{{]*\)\s*\{{')
code+='''
int main(void){
 assert(!esb_relay_block_mask(15,1));
 assert(esb_relay_block_mask(0x8000,1));assert(relay_block_drop(15,1));
 assert(esb_relay_block_mask(0,0));
 assert(!esb_relay_block_mask(8,0));
 assert(!esb_relay_block_mask(8,61));
 now=0xfffffff0;
 assert(esb_relay_block_mask((1<<3)|(1<<9),2));
 for(unsigned i=0;i<16;i++) assert(relay_block_drop(i,5)==(i==3||i==9));
 now+=1999; assert(relay_block_drop(3,1));
 now++; assert(!relay_block_drop(3,1));
 assert(esb_relay_block_mask((1<<3)|(1<<6)|(1<<9),60));
 assert(relay_block_drop(6,1));
 esb_relay_block(9,0);
 assert(!relay_block_drop(3,1) && !relay_block_drop(6,1));
 esb_relay_block(3,100);
 assert(relay_block_drop(3,0) && !relay_block_drop(9,0));
 assert(esb_relay_block_mask(0,0));
 assert(!relay_block_drop(3,0));
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d);(p/'test.c').write_text(code)
 subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-Wall','-Wextra','-Werror','-I'+str(here.parents[2]/'src/connection'),str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
assert '&& !relay_multi_blocked(id)' in src
print('Multi-block: mask bounds, timeout rollover, early-receipt exclusion and finally restoration passed')
