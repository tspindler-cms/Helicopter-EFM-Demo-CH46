"""Build the single governor-enabled Windows x64 update. Requires ziglang==0.13.0."""
from pathlib import Path
import subprocess,sys,shutil
p=Path(__file__).resolve().parent;s=p/'EFM Source Code/EFM'
out=p/'bin/CH46D.dll';out.parent.mkdir(parents=True,exist_ok=True)
subprocess.run([sys.executable,'-m','ziglang','c++','-target','x86_64-windows-gnu',
 '-std=c++17','-O2','-shared','-DFM_EXPORTS','-D_USE_MATH_DEFINES','-DNOMINMAX',
 '-I'+str(s),'-I'+str(s/'include'),str(s/'CH46D.cpp'),str(s/'FlightModel/Aero.cpp'),
 str(s/'dllmain.cpp'),'-o',str(out)],check=True)
# Developer symbols/import library are not part of the DCS installation.
(p/'symbols').mkdir(exist_ok=True)
for ext in ['pdb','lib']:
 product=out.with_suffix('.'+ext)
 if product.exists(): shutil.move(str(product),str(p/'symbols'/product.name))
print(out)
