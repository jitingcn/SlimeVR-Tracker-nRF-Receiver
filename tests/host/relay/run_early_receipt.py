"""Check production early-receipt validation, including truncated composites."""
from pathlib import Path as _RelayPath
import sys as _relay_sys
_relay_sys.path.insert(0, str(_RelayPath(__file__).resolve().parents[1] / 'harness'))
from relay_config import select_relay

import os
from pathlib import Path
import subprocess
import sys
import tempfile
here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[3] / 'tracker/tests/host/harness/python'))
from c_extract import extract_block
src = select_relay((here.parents[2] / 'src/connection/esb.c').read_text())
functions = '\n'.join(extract_block(src, rf'^static (?:int|bool) {name}\([^;{{]*\)\s*\{{')
    for name in ('composite_body_length', 'relay_early_receipt_valid'))
code = '''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define ESB_COMPOSITE_TYPE 0x10
#define TRACKER_EVENT_ESB_TYPE 0xe0
#define TRACKER_EVENT_BODY_LEN 12
''' + functions + '''
int main(void) {
 uint8_t p[72] = {1, 10};
 assert(relay_early_receipt_valid(p,17));
 assert(!relay_early_receipt_valid(p,16));
 p[0]=ESB_COMPOSITE_TYPE; p[2]=1; p[3]=2;
 assert(relay_early_receipt_valid(p,18));
 for (unsigned n=0;n<18;n++) assert(!relay_early_receipt_valid(p,n));
 assert(!relay_early_receipt_valid(p,19));
 p[2]=0; assert(!relay_early_receipt_valid(p,18));
 p[2]=2; assert(!relay_early_receipt_valid(p,18));
 p[2]=1; p[3]=0xe0; assert(!relay_early_receipt_valid(p,18));
 p[3]=255; assert(!relay_early_receipt_valid(p,18));
}
'''
with tempfile.TemporaryDirectory() as d:
 p = Path(d); (p/'test.c').write_text(code)
 subprocess.run([os.environ.get('CC','cc'),'-std=c11','-Wall','-Wextra','-Werror',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('Early RF receipt: valid records, truncated lists, events and unknown records passed')
