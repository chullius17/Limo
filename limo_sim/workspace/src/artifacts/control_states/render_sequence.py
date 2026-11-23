"""Control-state diagram, preserving the supplied diagram's visual style."""
from pathlib import Path
from html import escape
import math
from PIL import Image, ImageDraw, ImageFont
OUT=Path(__file__).resolve().parent
W,H=1500,2180
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


XS=[110,390,730,1080,1390]
NAMES=['GUI','path_executor','Controller Server','AckermannMPCController','cmd_vel_mux']
for x,name in zip(XS,NAMES):
 box(x-95,20,190,48,'#e5f2ff','#79b8ff',0)
 text(x,44,name,13,'#555555')
 d.line([(x*S,68*S),(x*S,2090*S)],fill='#999999',width=S)
 svg.append(f'<path d="M {x} 68 V 2090" stroke="#999999" stroke-width="0.8"/>')
 box(x-95,2090,190,48,'#e5f2ff','#79b8ff',0)
 text(x,2114,name,13,'#555555')

def phase(y,name):
 box(85,y,1330,29,'#fff7ef','#edaa7d',0)
 text(750,y+15,name,13,'#666666')
def msg(a,b,y,s,reply=False):
 arrow([(XS[a],y),(XS[b],y)],reply)
 text((XS[a]+XS[b])/2,y-12,s,12,'#666666')
def own(a,y,s):
 x=XS[a]
 arrow([(x,y),(x+42,y),(x+42,y+20),(x,y+20)])
 text(x+52,y+7,s,12,'#666666') if False else None
 # Place long self-call labels above the loop.
 text(x+75,y-13,s,12,'#666666')
def note(x,y,s):text(x,y,s,12,'#777777')

phase(90,'START')
msg(0,1,150,'/limo/control/set_active (true)')
msg(1,0,195,'WAITING · /limo/control/status',True)
own(1,245,'Disponibilità servizi e controlli di partenza')
note(600,290,'Verifica footprint nel tubo, se richiesta; errore / timeout → ERROR')
msg(1,2,335,'send_goal_async(percorso, controller_id)')
msg(2,1,375,'Goal accettato',True)
msg(1,0,415,'ACTIVE · /limo/control/status',True)
msg(2,3,455,'setPlan(percorso)')
box(710,480,750,195,'#f5f5f5','#dddddd',0)
text(745,494,'loop',12,'#777777')
text(1080,494,'[Ciclo di controllo]',12,'#777777')
msg(2,3,535,'computeVelocityCommands(pose, velocità)')
msg(3,2,575,'Primo comando della soluzione MPC',True)
msg(2,4,615,'/cmd_vel_autonomy')
own(4,646,'')
note(1245,656,'Selezione sorgente → /cmd_vel')
msg(2,1,700,'Feedback: distanza e velocità',True)

phase(740,'PAUSE')
msg(0,1,795,'/limo/control/set_enabled (false)')
msg(1,2,840,'cancel_goal_async()')
msg(1,0,885,'PAUSED · /limo/control/status',True)
msg(2,4,930,'Comando nullo su /cmd_vel_autonomy')
msg(2,1,975,'Risultato: goal cancellato',True)
note(600,1005,'PAUSED — percorso conservato per RESUME')

phase(1035,'RESUME')
msg(0,1,1090,'/limo/control/set_enabled (true)')
msg(1,0,1135,'WAITING · /limo/control/status',True)
own(1,1175,'Ripete i controlli necessari')
msg(1,2,1230,'send_goal_async(percorso, controller_id)')
msg(2,1,1270,'Goal accettato',True)
msg(1,0,1310,'ACTIVE · /limo/control/status',True)

phase(1350,'ABORT')
msg(0,1,1405,'/limo/control/set_active (false)')
own(1,1445,'Cancella percorso e richiesta di controllo')
msg(1,2,1500,'cancel_goal_async(), se azione attiva')
msg(1,0,1540,'IDLE · /limo/control/status',True)
note(600,1570,'IDLE pubblicato subito, senza attendere la conferma di cancellazione')
msg(2,4,1610,'Comando nullo su /cmd_vel_autonomy')
msg(2,1,1650,'Risultato tardivo: nessun ripristino del percorso',True)
note(650,1685,'Per ripartire: ricevere un nuovo percorso → READY, poi START')

phase(1725,'SUCCESS — scenario alternativo durante ACTIVE')
own(1,1785,'Verifica arrivo tramite odometria e TF')
note(660,1830,'Centro nel cerchio del goal e robot fermo per il tempo configurato')
msg(1,2,1875,'cancel_goal_async(), se azione ancora attiva')
msg(1,0,1915,'SUCCESS · /limo/control/status',True)
msg(2,4,1955,'Comando nullo, se azione cancellata')
msg(2,1,1995,'Risultato tardivo: SUCCESS conservato',True)
note(650,2040,'Se FollowPath termina prima, resta ACTIVE finché arrivo e arresto sono confermati')
note(750,2160,'Il mux applica priorità e timeout: teleop ha precedenza sui comandi autonomi.')
svg.append('</svg>')
(OUT/'sequenza_controllo.svg').write_text('\n'.join(svg),encoding='utf-8')
im.save(OUT/'sequenza_controllo.png')
