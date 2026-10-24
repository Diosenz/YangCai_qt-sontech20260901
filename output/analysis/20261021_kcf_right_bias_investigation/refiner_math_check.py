"""Independent numeric check of new local model; not a compiled C++ test."""
import json
from pathlib import Path
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

def extract(im, cx, cy, radius=100):
    xb, xe = max(0, round(cx)-radius), min(im.shape[1], round(cx)+radius+1)
    yb, ye = max(0, round(cy)-60), min(im.shape[0], round(cy)+61)
    result=[]
    for x in range(xb,xe):
        col=im[yb:ye,x].astype(float)
        bg=col.min()
        smooth=(col[:-2]+col[1:-1]+col[2:])/3
        p=int(smooth.argmax())+1
        peak=smooth[p-1]
        if peak-bg<40 or p<5 or len(col)-p<=5: continue
        threshold=bg+.5*(peak-bg)
        lo,hi=p-1,p+1
        while lo>=0 and col[lo]>threshold: lo-=1
        while hi<len(col) and col[hi]>threshold: hi+=1
        if hi-lo-1>12: continue
        q=np.arange(p-3,p+4)
        weight=np.maximum(col[q]-bg,0)
        result.append((x,yb+np.sum(q*weight)/weight.sum()))
    return np.array(result),xe-xb

def line(a,cx):
    x,y=a[:,0]-cx,a[:,1]
    design=np.column_stack([np.ones(len(x)),x])
    co=np.linalg.lstsq(design,y,rcond=None)[0]
    for _ in range(4):
        w=np.sqrt(np.minimum(1,2/np.maximum(abs(y-design@co),1e-9)))
        co=np.linalg.lstsq(design*w[:,None],y*w,rcond=None)[0]
    return co

def split(samples):
    candidates=[]
    for i in range(25,len(samples)-24):
        x=.5*(samples[i-1,0]+samples[i,0])
        left,right=line(samples[:i],x),line(samples[i:],x)
        if left[1]<=.05 or right[1]>=-.05: continue
        pred=np.r_[left[0]+left[1]*(samples[:i,0]-x),right[0]+right[1]*(samples[i:,0]-x)]
        e=samples[:,1]-pred
        ae=abs(e)
        score=np.mean(np.where(ae<=2,e*e,4*ae-4))
        inside=ae<=3
        rms=np.sqrt(np.mean(e[inside]**2)) if inside.any() else 0
        candidates.append(dict(x=x,score=score,left_y=left[0],right_y=right[0],rms=rms,support=inside.mean(),left_support=int(inside[:i].sum()),right_support=int(inside[i:].sum())))
    if not candidates: return None
    best=min(candidates,key=lambda a:a['score'])
    near=[a['x'] for a in candidates if a['score']<=best['score']+.01]
    best['uncertainty']=max(near)-min(near)
    return best

def check(im,cx,cy):
    samples,width=extract(im,cx,cy)
    if len(samples)<50 or len(samples)/width<.75:return dict(status='insufficient',columns=len(samples))
    fit=split(samples)
    if fit is None:return dict(status='inconsistent')
    inner=split(samples[abs(samples[:,0]-cx)<=85])
    fit['inner_x']=inner['x'] if inner else None
    fit['status']='pass_geometry'
    if fit['support']<.75 or fit['rms']>1.8 or min(fit['left_support'],fit['right_support'])<25:fit['status']='inconsistent'
    elif inner is None or abs(inner['x']-fit['x'])>3 or max(fit['uncertainty'],inner['uncertainty'])>6:fit['status']='unstable'
    elif abs(fit['left_y']-fit['right_y'])>3:fit['status']='ambiguous_height'
    return fit

def main():
    results=[]
    rows=json.loads((HERE/'results.json').read_text(encoding='utf-8'))
    run=ROOT/rows['run']
    for row in rows['samples']:
        im=np.array(Image.open(run/row['source']).convert('L'))
        result=check(im,row['raw_x'],row['raw_y'])
        results.append(dict(frame=row['frame'],coarse_x=row['raw_x'],**result))
    yy,xx=np.indices((240,320))
    for cx,cy in [(166,107),(132,113)]:
        center=110+np.where(xx<150,.28*(xx-150),-.32*(xx-150))
        im=np.clip(15+210*np.exp(-.5*((yy-center)/1.2)**2),0,255).astype(np.uint8)
        results.append(dict(synthetic=True,coarse=[cx,cy],**check(im,cx,cy)))
    (HERE/'refiner_math_check.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    print(json.dumps(results,indent=2))

if __name__=='__main__':main()
