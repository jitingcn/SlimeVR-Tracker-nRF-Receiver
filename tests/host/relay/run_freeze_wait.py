"""Execute the production callback's RTC wait with a stopped clock."""
from pathlib import Path
import subprocess,tempfile
s=(Path(__file__).resolve().parents[3]/'src/connection/esb.c').read_text()
a=s.index('\tunsigned polls = 65536;');b=s.index('\tunsigned key = irq_lock();',a)
code='''#include <stdint.h>
#include <errno.h>
#include <assert.h>
struct work {unsigned freeze_rtc;int result;};
static struct {volatile unsigned COUNTER;} rtc;
#define NRF_RTC1 (&rtc)
static int relay_freeze_done;
static void k_sem_give(int *p){++*p;}
static void wait(struct work *w){
'''+s[a:b]+'''}
int main(void){struct work w={.freeze_rtc=100};rtc.COUNTER=97;wait(&w);
assert(w.result==-ETIMEDOUT && relay_freeze_done==1);
rtc.COUNTER=100;w.result=0;wait(&w);assert(w.result==0 && relay_freeze_done==1);}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t);(p/'t.c').write_text(code);subprocess.run(['cc','-Wall','-Werror',str(p/'t.c'),'-o',str(p/'t')],check=True);subprocess.run([str(p/'t')],check=True)
print('Stopped RTC callback wait terminates; freeze tick unchanged')
