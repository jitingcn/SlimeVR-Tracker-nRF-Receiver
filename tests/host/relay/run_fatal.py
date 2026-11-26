"""Validate retained records at every interrupted publication step."""
from pathlib import Path
import sys,subprocess,tempfile
h=Path(__file__).resolve().parent
sys.path.insert(0,str(h.parent/'harness'))
from c_extract import extract_block
s=(h.parents[2]/'src/system/system.c').read_text()
struct=extract_block((h.parents[2]/'src/system/system.h').read_text(),r'^struct sys_fatal_record\s*\{',True)
check=extract_block(s,r'^static uint32_t fatal_check\([^;{]*\)\s*\{')
read=extract_block(s,r'^const struct sys_fatal_record \*sys_last_fatal\([^;{]*\)\s*\{')
a=s.index('\tfatal_record.magic = 0;',s.index('void k_sys_fatal_error_handler'))
b=s.index('#if ADAFRUIT_RECOVERY_ENABLED',a)
steps=[line for line in s[a:b].splitlines() if line.strip()]
prefix='''#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#define __DMB() ((void)0)
#define FATAL_RECORD_MAGIC 0xFA7A1C0DU
'''+struct+'\nstatic struct sys_fatal_record fatal_record;\n'+check+read+'''
struct arch_esf { struct {uint32_t pc,lr;} basic; };
static int k_is_pre_kernel(void){return 0;}
static void *k_current_get(void){return (void*)42;}
int main(void){unsigned reason=7;uint32_t boot_reset_reason=4;
struct arch_esf e={{123,456}},*esf=&e;
'''
body=''
for step in steps:
 body+=step+'\n'
 if 'magic = FATAL_RECORD_MAGIC' not in step:body+='assert(!sys_last_fatal());\n'
body+='assert(sys_last_fatal());fatal_record.lr++;assert(!sys_last_fatal());}\n'
with tempfile.TemporaryDirectory() as t:
 p=Path(t)/'f.c';p.write_text(prefix+body);e=Path(t)/'f'
 subprocess.run(['cc','-Wall','-Wextra','-Werror',str(p),'-o',str(e)],check=True);subprocess.run([str(e)],check=True)
print('Fatal retention: all partial writes rejected; complete record validated; corruption rejected')
