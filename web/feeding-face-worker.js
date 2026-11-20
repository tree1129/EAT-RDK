// Classic worker: MediaPipe's WASM loader uses importScripts.
importScripts('./vendor/face/vision_bundle.js');
let landmarker;
(async()=>{
 try {
  const files=await Vision.FilesetResolver.forVisionTasks(new URL('./vendor/face/wasm',self.location).href);
  landmarker=await Vision.FaceLandmarker.createFromOptions(files,{
   baseOptions:{modelAssetPath:new URL('./vendor/face/face_landmarker.task',self.location).href,delegate:'CPU'},
   runningMode:'VIDEO',numFaces:2,minFaceDetectionConfidence:.65,minFacePresenceConfidence:.65,minTrackingConfidence:.65
  });
  const contours={};
  for(const key of ['FACE_LANDMARKS_TESSELATION','FACE_LANDMARKS_FACE_OVAL','FACE_LANDMARKS_LIPS','FACE_LANDMARKS_LEFT_EYE','FACE_LANDMARKS_RIGHT_EYE','FACE_LANDMARKS_LEFT_EYEBROW','FACE_LANDMARKS_RIGHT_EYEBROW','FACE_LANDMARKS_LEFT_IRIS','FACE_LANDMARKS_RIGHT_IRIS'])contours[key]=Vision.FaceLandmarker[key];
  self.postMessage({type:'ready',contours});
 }catch(error){self.postMessage({type:'error',message:String(error)});}
})();
self.onmessage=event=>{
 const {bitmap,id,timestamp}=event.data;
 try {
  const start=performance.now();
  const result=landmarker.detectForVideo(bitmap,timestamp);
  self.postMessage({type:'result',id,faceCount:result.faceLandmarks.length,points:result.faceLandmarks[0]||[],ms:performance.now()-start});
 }catch(error){self.postMessage({type:'error',id,message:String(error)});}
 finally{bitmap.close();}
};
