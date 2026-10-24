"""Independent read-only translation of branch math; not native estimator replay."""
import json,math
from pathlib import Path
import numpy as np
from PIL import Image
ROOT=Path.cwd();BASE=Path(__file__).resolve().parent

def peaks(im,x,expected,radius):
 b=max(2,math.floor(expected-radius));e=min(im.shape[0]-3,math.ceil(expected+radius))
 if b>=e:return []
 bg=float(im[b:e+1,x].min());out=[]
 smooth=lambda y:float(im[y-1:y+2,x].sum())/3.
 for y in range(b,e+1):
  z=smooth(y)
  if z-bg<40 or z<smooth(y-1) or z<=smooth(y+1):continue
  threshold=bg+(z-bg)*.5;left=y;right=y
  while left>b and im[left-1,x]>threshold:left-=1
  while right<e and im[right+1,x]>threshold:right+=1
  if right-left+1>12:continue
  rows=np.arange(max(0,y-3),min(im.shape[0]-1,y+3)+1);w=np.maximum(0,im[rows,x]-bg)
  if w.sum()>0:out.append((float((w*rows).sum()/w.sum()),z-bg))
 return out

def branch(im,pred,side,slope,height,radius):
 samples=[];amb=0
 for offset in range(35,91):
  x=math.floor(pred[0]+.5)+side*offset
  if not 0<=x<im.shape[1]:continue
  expected=pred[1]+height+slope*(x-pred[0]);ps=sorted(peaks(im,x,expected,radius),key=lambda a:abs(a[0]-expected))
  if not ps:continue
  if any(abs(abs(z[0]-expected)-abs(ps[0][0]-expected))<1.5 and abs(z[0]-ps[0][0])>=7 for z in ps[1:]):amb+=1;continue
  if abs(ps[0][0]-expected)>radius:continue
  samples.append((x-pred[0],ps[0][0]))
 result={'count':len(samples),'ambiguous_columns':amb,'valid':False}
 if len(samples)<20:return result
 data=np.array(samples);x=data[:,0];y=data[:,1];m=slope;c=pred[1]+height
 for step in range(4):
  error=y-(c+m*x);w=np.ones(len(x)) if not step else np.minimum(1,1.5/np.maximum(abs(error),1e-9));sw=w.sum();sx=(w*x).sum();sy=(w*y).sum();sxx=(w*x*x).sum();sxy=(w*x*y).sum();det=sw*sxx-sx*sx
  if det<=1e-9:return result
  m=(sw*sxy-sx*sy)/det;c=(sy-m*sx)/sw
 rms=float(np.sqrt(np.mean((y-c-m*x)**2)));result.update(slope=float(m),intercept=float(c),rms=rms,valid=bool(rms<=1.8 and abs(m-slope)<=.25));return result

anchor=np.array(Image.open(ROOT/'tests/fixtures/online_image_geometry/frame_14591.png').convert('L'),float);target=np.array(Image.open(ROOT/'tests/fixtures/online_image_geometry/frame_15248.png').convert('L'),float);p=np.array([435.5,608.562989]);left_height=612.895397-p[1];right_height=604.230580-p[1]
l=branch(anchor,p,-1,.45,left_height,20);r=branch(anchor,p,1,-.45,right_height,20);result={'scope':'Independent read-only branch math, not native estimator/robot replay','anchor_left':l,'anchor_right':r}
if l['valid'] and r['valid']:
 tl=branch(target,p,-1,l['slope'],left_height,7);tr=branch(target,p,1,r['slope'],right_height,7);result.update(target_left=tl,target_right=tr)
 if tl['valid'] and tr['valid']:
  gap=right_height-left_height;dx=(tr['intercept']-tl['intercept']-gap)/(tl['slope']-tr['slope']);point=[float(p[0]+dx),float(.5*(tl['intercept']+tr['intercept']+(tl['slope']+tr['slope'])*dx))];result['target_supported_point']=point;result['innovation_px']=float(np.linalg.norm(np.array(point)-p));result['distance_to_visual_landmark_px']=float(np.linalg.norm(np.array(point)-[425,607]));result['slope_changes']=[abs(tl['slope']-l['slope']),abs(tr['slope']-r['slope'])]
with (BASE/'pixel_branch_review.json').open('x',encoding='utf-8') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
