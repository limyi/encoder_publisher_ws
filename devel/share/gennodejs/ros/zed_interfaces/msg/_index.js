
"use strict";

let Keypoint2Di = require('./Keypoint2Di.js');
let Skeleton2D = require('./Skeleton2D.js');
let Object = require('./Object.js');
let BoundingBox3D = require('./BoundingBox3D.js');
let Keypoint3D = require('./Keypoint3D.js');
let BoundingBox2Df = require('./BoundingBox2Df.js');
let BoundingBox2Di = require('./BoundingBox2Di.js');
let Keypoint2Df = require('./Keypoint2Df.js');
let Skeleton3D = require('./Skeleton3D.js');
let ObjectsStamped = require('./ObjectsStamped.js');
let RGBDSensors = require('./RGBDSensors.js');

module.exports = {
  Keypoint2Di: Keypoint2Di,
  Skeleton2D: Skeleton2D,
  Object: Object,
  BoundingBox3D: BoundingBox3D,
  Keypoint3D: Keypoint3D,
  BoundingBox2Df: BoundingBox2Df,
  BoundingBox2Di: BoundingBox2Di,
  Keypoint2Df: Keypoint2Df,
  Skeleton3D: Skeleton3D,
  ObjectsStamped: ObjectsStamped,
  RGBDSensors: RGBDSensors,
};
