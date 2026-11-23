"""Control-state diagram, preserving the supplied diagram's visual style."""
from pathlib import Path
from html import escape
import math
from PIL import Image, ImageDraw, ImageFont
OUT=Path(__file__).resolve().parent
W,H=1400,1420
S=2
im=Image.new('RGB',(W*S,H*S),'#f5f5f5'); d=ImageDraw.Draw(im)
svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">','<rect width="100%" height="100%" fill="#f5f5f5"/>']
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'; BOLD='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'

def text(x,y,s,size=13,color='#183c69',bold=False):
 f=ImageFont.truetype(BOLD if bold else FONT,round(size*S))
 d.text((x*S,y*S),s,font=f,fill=color,anchor='mm')
 svg.append(f'<text x="{x}" y="{y}" font-family="Arial,sans-serif" font-size="{size}" font-weight="{600 if bold else 400}" fill="{color}" text-anchor="middle" dominant-baseline="central">{escape(s)}</text>')
def box(x,y,w,h,fill,stroke,r=18):
 d.rounded_rectangle((x*S,y*S,(x+w)*S,(y+h)*S),radius=r*S,fill=fill,outline=stroke,width=1*S)
 svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{fill}" stroke="{stroke}" stroke-width="0.8"/>')
def arrow(pts,dash=False):
 col='#999999'
 # Round each orthogonal corner using quadratic curves in the SVG.
 path=f'M {pts[0][0]} {pts[0][1]}'
 for i in range(1,len(pts)-1):
  a,b,c=pts[i-1],pts[i],pts[i+1]; r=min(7,math.dist(a,b)/2,math.dist(b,c)/2)
  u=(b[0]+(a[0]-b[0])*r/math.dist(a,b),b[1]+(a[1]-b[1])*r/math.dist(a,b))
  v=(b[0]+(c[0]-b[0])*r/math.dist(b,c),b[1]+(c[1]-b[1])*r/math.dist(b,c))
  path+=f' L {u[0]} {u[1]} Q {b[0]} {b[1]} {v[0]} {v[1]}'
 path+=f' L {pts[-1][0]} {pts[-1][1]}'
 svg.append(f'<path d="{path}" fill="none" stroke="{col}" stroke-width="0.8"'+(' stroke-dasharray="2 2"' if dash else '')+'/>')
 for a,b in zip(pts,pts[1:]):
  length=math.dist(a,b)
  if dash:
   for n in range(0,int(length*S),4*S):
    p=[((a[0]+(b[0]-a[0])*t/(length*S))*S,(a[1]+(b[1]-a[1])*t/(length*S))*S) for t in (n,min(n+2*S,length*S))]
    d.line(p,fill=col,width=S)
  else:d.line([(x*S,y*S) for x,y in (a,b)],fill=col,width=S)
 a,b=pts[-2:]; t=math.atan2(b[1]-a[1],b[0]-a[0]); ends=[(b[0]-6*math.cos(t-.55),b[1]-6*math.sin(t-.55)),b,(b[0]-6*math.cos(t+.55),b[1]-6*math.sin(t+.55))]
 d.line([(x*S,y*S) for x,y in ends],fill=col,width=S)
 svg.append('<polyline points="'+' '.join(f'{x},{y}' for x,y in ends)+f'" fill="none" stroke="{col}" stroke-width="0.8"/>')
def label(x,y,s,size=12):
 f=ImageFont.truetype(BOLD,round(size*S)); w=d.textlength(s,font=f)/S+24
 box(x-w/2,y-13,w,26,'#f4f9ff','#cbdced',13)
 text(x,y,s,size,'#0056a6',True)
def node(x,y,name,kind='blue'):
 fill,edge,col={'blue':('#e5efff','#628de0','#1c3d69'),'yellow':('#fff4d8','#cf9b3b','#60491d'),'green':('#e5f4ea','#69a481','#285c40'),'red':('#fdeaea','#da7777','#843030')}[kind]
 box(x,y,132,60,fill,edge,17);text(x+66,y+31,name,14,col,True)


XS=[100,380,700,1020,1300]
NAMES=['GUI','path_executor','Controller Server','AckermannMPCController','cmd_vel_mux']
for x,name in zip(XS,NAMES):
 box(x-90,20,180,44,'#e5f2ff','#79b8ff',0)
 text(x,42,name,12,'#555555')
 d.line([(x*S,64*S),(x*S,1340*S)],fill='#999999',width=S)
 svg.append(f'<path d="M {x} 64 V 1340" stroke="#999999" stroke-width="0.8"/>')
 box(x-90,1340,180,44,'#e5f2ff','#79b8ff',0)
 text(x,1362,name,12,'#555555')

def phase(y,name):
 box(75,y,1250,27,'#fff7ef','#edaa7d',0)
 text(700,y+14,name,13,'#666666')
def msg(a,b,y,s,reply=False):
 arrow([(XS[a],y),(XS[b],y)],reply)
 text((XS[a]+XS[b])/2,y-12,s,13,'#666666')
def note(x,y,s):text(x,y,s,13,'#666666')

phase(85,'START')
msg(0,1,143,'START')
note(380,181,'WAITING — controlli di partenza')
msg(1,2,227,'Invia percorso')
msg(2,1,270,'Goal accettato → ACTIVE',True)
msg(2,3,313,'Imposta percorso')

box(680,338,690,174,'#f5f5f5','#dddddd',0)
note(1025,355,'loop — ciclo di controllo')
msg(2,3,398,'Calcola comando MPC')
msg(3,2,439,'Comando di velocità',True)
msg(2,4,483,'/cmd_vel_autonomy → selezione sorgente → /cmd_vel')

phase(545,'PAUSE / RESUME')
msg(0,1,603,'PAUSE')
msg(1,2,647,'Richiede cancellazione → arresto autonomo')
note(380,687,'PAUSED — percorso conservato')
msg(0,1,736,'RESUME')
note(380,775,'WAITING — ripete i controlli')
msg(1,2,819,'Reinvia percorso')
msg(2,1,862,'Goal accettato → ACTIVE',True)

phase(900,'ABORT — alternativa durante l’esecuzione')
msg(0,1,958,'ABORT')
note(380,998,'IDLE — percorso cancellato')
msg(1,2,1043,'Richiede cancellazione → arresto autonomo')
note(700,1084,'Per ripartire: nuovo percorso → READY → START')

phase(1120,'SUCCESS — alternativa durante l’esecuzione')
note(530,1180,'Centro nel goal e robot fermo per il tempo configurato')
msg(1,0,1225,'SUCCESS',True)
msg(1,2,1270,'Cancella l’azione, se ancora attiva')
note(700,1311,'La sola conclusione del controller non basta a confermare SUCCESS')

note(700,1406,'Controlli o esecuzione falliti → ERROR   ·   Il mux dà precedenza alla teleoperazione')
svg.append('</svg>')
(OUT/'sequenza_controllo_semplice.svg').write_text('\n'.join(svg),encoding='utf-8')
im.save(OUT/'sequenza_controllo_semplice.png')
