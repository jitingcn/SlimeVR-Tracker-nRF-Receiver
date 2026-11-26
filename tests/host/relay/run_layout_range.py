from pathlib import Path
import os,shlex,subprocess,tempfile
here=Path(__file__).resolve().parent
root=here.parents[2]
with tempfile.TemporaryDirectory() as d:
 p=Path(d)
 for direct,repairs,slot,tps in [(16,r,16,100) for r in range(0,6)]+[(10,1,16,90),(16,1,16,120)]:
  config=p/'config.h';config.write_text(f'#define RELAY_HOST_DIRECT_SLOTS {direct}U\n#define RELAY_HOST_REPAIR_SLOTS {repairs}U\n#define RELAY_HOST_SLOT_TICKS {slot}U\n#define RELAY_HOST_TPS_MAX {tps}U\n')
  subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-include',str(config),'-I',str(root/'src/connection'),str(here/'test_layout_range.c'),str(root/'src/connection/relay_coord.c'),'-o',str(p/'test')],check=True)
  subprocess.run([str(p/'test')],check=True)
