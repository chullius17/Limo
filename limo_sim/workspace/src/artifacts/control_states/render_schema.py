"""Control-state diagram, preserving the supplied diagram's visual style."""
from pathlib import Path
from html import escape
import math
from PIL import Image, ImageDraw, ImageFont
OUT=Path(__file__).resolve().parent
W,H=1000,1050
S=2
im=Image.new('RGB',(W*S,H*S),'white'); d=ImageDraw.Draw(im)
svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">','<rect width="100%" height="100%" fill="white"/>']
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

# Main chain and the pause branch retain the sparse layout of the reference.
node(574,48,'IDLE')
node(574,218,'READY')
node(254,398,'WAITING')
node(254,658,'ACTIVE')
node(254,918,'SUCCESS','green')
node(54,218,'PAUSED','yellow')
node(574,658,'ERROR','red')

arrow([(640,108),(640,218)])
label(640,163,'Percorso ricevuto')
arrow([(574,248),(320,248),(320,398)])
label(438,248,'START')
arrow([(320,458),(320,658)])
label(320,544,'Controlli superati e goal accettato')
arrow([(320,718),(320,918)])
label(320,810,'Arrivo confermato: fermo nel cerchio del goal')

arrow([(254,688),(120,688),(120,278)])
label(120,586,'PAUSE')
arrow([(254,428),(154,428),(154,278)])
label(154,369,'PAUSE')
arrow([(186,248),(216,248),(216,353),(292,353),(292,398)])
label(216,319,'RESUME')

arrow([(386,428),(640,428),(640,658)])
label(640,505,'Validazione, invio o accettazione falliti')
arrow([(386,688),(574,688)])
label(480,688,'Esecuzione fallita')

# Restarts preserve the stored path. Common path events are compact labels.
arrow([(640,718),(640,851),(486,851),(486,458),(360,458)])
label(486,772,'START')
arrow([(386,948),(526,948),(526,378),(360,378),(360,398)])
label(526,900,'START')

# A common connector avoids duplicating ABORT and path-replacement lines.
arrow([(846,536),(846,78),(706,78)],True)
label(846,303,'ABORT')
label(846,337,'Percorso cancellato')
text(846,557,'Da qualsiasi stato',12,'#666666')
text(846,590,'Nuovo percorso → READY',12)
text(500,1021,'Percorso vuoto → IDLE   ·   SUCCESS resta memorizzato fino a nuovo START, percorso o ABORT',11,'#777777')
svg.append('</svg>')
(OUT/'schema_stati_controllo.svg').write_text('\n'.join(svg),encoding='utf-8')
im.save(OUT/'schema_stati_controllo.png')
