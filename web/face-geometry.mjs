// Pixel geometry only; these values are not camera-space depth or robot targets.
export function mouthGeometry(points,width,height) {
  const ids=[61,291,13,14];
  if(!points || !ids.every(i=>points[i] && Number.isFinite(points[i].x) && Number.isFinite(points[i].y)))return null;
  const pixel=i=>({x:points[i].x*width,y:points[i].y*height});
  const [left,right,top,bottom]=ids.map(pixel);
  const distance=(a,b)=>Math.hypot(a.x-b.x,a.y-b.y);
  const span=distance(left,right);
  if(span<2)return null;
  const ratio=distance(top,bottom)/span;
  return {center:{x:(top.x+bottom.x)/2,y:(top.y+bottom.y)/2},ratio,state:ratio>.12?'张开':ratio<.06?'闭合':'微张'};
}
