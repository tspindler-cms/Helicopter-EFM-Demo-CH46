"""Restricted-model checks; not a DCS or real-aircraft flight validation."""
from pathlib import Path
import subprocess,tempfile,re,shutil,sys
s=Path(__file__).resolve().parents[1]/'EFM Source Code/EFM'
tests=Path(__file__).parent
with tempfile.TemporaryDirectory() as tmp:
 def build(name,flags=(),aero=False):
  exe=Path(tmp)/(name+('_off' if flags else '_on'))
  compiler=['g++'] if shutil.which('g++') else [sys.executable,'-m','ziglang','c++']
  cmd=[*compiler,'-std=c++17','-O2','-D_USE_MATH_DEFINES','-DCH46_HEADLESS_TEST','-I'+str(s),'-I'+str(s/'include'),*flags,str(tests/(name+'.cpp'))]
  if aero:cmd.append(str(s/'FlightModel/Aero.cpp'))
  subprocess.run(cmd+['-o',str(exe)],check=True)
  return exe
 for flags in [(),('-DCH46_RATE_DAMPING_OFF',)]:
  subprocess.run([str(build('rate_feedback_tests',flags))],check=True)
 subprocess.run([str(build('debug_observer_test'))],check=True)
 vals={}
 for mode,flags in [('off',('-DCH46_RATE_DAMPING_OFF',)),('on',())]:
  exe=build('lateral_response',flags,True)
  for dt in ([.003] if mode=='off' else [.006,.003,.0015]):
   r=subprocess.run([str(exe),str(dt)],check=True,capture_output=True,text=True)
   m=re.search(r'maxrate=([\d.e+-]+) tailrate=([\d.e+-]+)',r.stderr)
   assert m,r.stderr
   vals[mode,dt]=tuple(map(float,m.groups()))
   print(mode,dt,'peak/tail deg/s',vals[mode,dt])
 assert vals['on',.003][0]<.5*vals['off',.003][0]
 assert vals['on',.003][1]<.5*vals['off',.003][1]
 assert abs(vals['on',.006][0]/vals['on',.0015][0]-1)<.05
 print('PASS restricted lateral-response comparison and timestep check')
