"""Exercise the actual admission function with the sixteen-ID coordinator."""
from pathlib import Path
import os, shlex, subprocess, sys, tempfile
here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parent / 'harness'))
from c_extract import extract_block
source = (here.parents[2] / 'src/connection/esb.c').read_text()
admit = extract_block(source, r'^static uint32_t tdma_admissible_mask\([^;{]*\)\s*\{')
prefix = '''
#include "relay_coord.h"
#include <assert.h>
#define BIT(n) (1U << (n))
#define MAX_TRACKERS 16
static unsigned stored_trackers = 16;
static struct relay_coord relay_coord;
static struct relay_runtime_settings relay_settings = {.max_trackers = 16};
'''
cases = '''
int main(void) {
 relay_coord_init(&relay_coord); relay_coord.v3_requested = true;
 for (unsigned id = 0; id < RC_MAX; id++)
  relay_coord_note_caps(&relay_coord, id, (RELAY_V3_VERSION << RELAY_CAP_VERSION_SHIFT) | RELAY_CAP_HELPER);
 assert(tdma_admissible_mask(0x3ff) == 0x3ff);
 assert(relay_coord_v3_ready(&relay_coord, 0x3ff));
 for (unsigned id = 10; id < 16; id++) {
  unsigned desired = 3 | BIT(id);
  assert(tdma_admissible_mask(desired) == desired);
  assert(relay_coord_v3_ready(&relay_coord, tdma_admissible_mask(desired)));
 }
 assert(tdma_admissible_mask(0xffff) == 0xffff);
 assert(relay_coord_v3_ready(&relay_coord, 0xffff));
 return 0;
}
'''
with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp)
    (p/'test.c').write_text(prefix + admit + cases)
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + ['-std=c11', '-Wall', '-Wextra', '-Werror',
        '-fsanitize=address,undefined',
        '-I'+str(here.parents[2]/'src/connection'), str(p/'test.c'),
        str(here.parents[2]/'src/connection/relay_coord.c'), '-o', str(p/'test')], check=True)
    subprocess.run([str(p/'test')], check=True)
print('capacity: all 16 IDs retain service and can enter v3')
