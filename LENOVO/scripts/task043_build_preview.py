#!/usr/bin/env python3
"""Build the bounded UI preview package for one saved binary PCD map."""
from __future__ import annotations
import hashlib, json, os, struct, sys
from pathlib import Path
import numpy as np
from PIL import Image

def digest(path):
 h=hashlib.sha256();
 with open(path,'rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
 return h.hexdigest()

def read_pcd(path):
 with open(path,'rb') as f:
  header=[]
  while True:
   line=f.readline()
   if not line: raise ValueError('PCD missing DATA')
   text=line.decode('ascii').strip(); header.append(text)
   if text.startswith('DATA '): break
  fields=next(x.split()[1:] for x in header if x.startswith('FIELDS '))
  sizes=list(map(int,next(x.split()[1:] for x in header if x.startswith('SIZE '))))
  types=next(x.split()[1:] for x in header if x.startswith('TYPE '))
  points=int(next(x.split()[1] for x in header if x.startswith('POINTS ')))
  if [fields[i] for i in range(3)] != ['x','y','z'] or types[:3] != ['F']*3 or sizes[:3] != [4]*3:
   raise ValueError('unsupported PCD xyz layout')
  if next(x for x in header if x.startswith('DATA ')) != 'DATA binary': raise ValueError('binary PCD required')
  offset=sum(sizes); payload=f.read()
  if len(payload) < points*offset: raise ValueError('short PCD payload')
  dtype=np.dtype([(n,'<f4') for n,s,t in zip(fields,sizes,types)])
  arr=np.frombuffer(payload,dtype=dtype,count=points)
  return np.column_stack((arr['x'],arr['y'],arr['z']))

def main():
 pcd=Path(sys.argv[1]).resolve(); ikd=Path(sys.argv[2]).resolve(); package=Path(sys.argv[3]).resolve(); name=sys.argv[4]
 points=read_pcd(pcd)
 if len(points)==0 or not np.isfinite(points).all(): raise SystemExit('invalid points')
 bounds=np.concatenate((points.min(0),points.max(0)))
 xmin,ymin,zmin,xmax,ymax,zmax=map(float,bounds)
 width,height=1200,900; zlo,zhi=.4,2.0
 mask=(points[:,2]>=zlo)&(points[:,2]<=zhi); xy=points[mask,:2]
 if len(xy)==0: raise SystemExit('no points in preview slice')
 hist=np.zeros((height,width),dtype=np.uint32)
 preview_xmin, preview_ymin = map(float, xy.min(axis=0))
 preview_xmax, preview_ymax = map(float, xy.max(axis=0))
 if preview_xmax <= preview_xmin or preview_ymax <= preview_ymin: raise SystemExit("preview slice has zero XY extent")
 cols=np.clip(((xy[:,0]-preview_xmin)/(preview_xmax-preview_xmin)*width).astype(np.int64),0,width-1)
 rows=np.clip(((preview_ymax-xy[:,1])/(preview_ymax-preview_ymin)*height).astype(np.int64),0,height-1)
 np.add.at(hist,(rows,cols),1)
 pixels=np.full((height,width,3),(13,17,23),dtype=np.uint8); pixels[hist>0]=(200,215,224); pixels[hist>=5]=(240,245,248)
 package.mkdir(parents=True,exist_ok=True); preview=package/'preview'; preview.mkdir(exist_ok=True); dest_pcd=package/'PCD/GlobalMap.pcd'; dest_pcd.parent.mkdir(exist_ok=True)
 if dest_pcd.exists(): dest_pcd.unlink()
 import shutil; shutil.copyfile(pcd,dest_pcd)
 dest_ikd=package/'PCD/GlobalMap_ikdtree.pcd'
 if not dest_ikd.exists(): shutil.copyfile(ikd,dest_ikd)
 png=preview/'map_preview.png'; Image.fromarray(pixels).save(png)
 pcd_sha=digest(dest_pcd); ikd_sha=digest(dest_ikd)
 map_id=f"{name}_{pcd_sha[:12]}"; manifest={'map_id':map_id,'map_version':1,'map_frame':'camera_init','pcd_sha256':pcd_sha,'ikdtree_sha256':ikd_sha,'point_count':int(len(points)),'map_bbox':{'x':[xmin,xmax],'y':[ymin,ymax],'z':[zmin,zmax]},'origin':'mapping_start','origin_pose':[0,0,0,0,0,0,1]}
 manifest_path=preview/'map_manifest.json'; manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
 metadata={'manifest_sha256':digest(manifest_path),'pcd_sha256':pcd_sha,'preview_sha256':digest(png),'world_bounds':[preview_xmin,preview_xmax,preview_ymin,preview_ymax],'z_slice':[zlo,zhi],'width_px':width,'height_px':height,'points_read':int(len(points)),'points_selected':int(len(xy))}
 metadata_path=preview/'preview_metadata.json'; metadata_path.write_text(json.dumps(metadata,indent=2)+'\n')
 candidate_pointer=package/'saved_map_preview.json'
 candidate_pointer.write_text(json.dumps({'metadata_path':str(metadata_path),'candidate':True})+'\n')
 print(json.dumps({'map_id':map_id,'map_version':1,'manifest':str(manifest_path),'candidate_preview':str(candidate_pointer)}))
if __name__=='__main__': main()
