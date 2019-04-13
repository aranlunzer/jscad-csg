/* eslint-disable semi */
const {parseOption, parseOptionAs3DVector, parseOptionAs2DVector, parseOptionAs3DVectorList, parseOptionAsFloat, parseOptionAsInt} = require('./optionParsers')
const {defaultResolution3D, defaultResolution2D, EPS} = require('../core/constants')
const Vector3 = require('../core/math/Vector3')
const Vertex3 = require('../core/math/Vertex3')
const Polygon3 = require('../core/math/Polygon3')
const {Connector} = require('../core/connectors')
const Properties = require('../core/Properties')
const {fromPolygons} = require('../core/CSGFactories')

/** Construct an axis-aligned solid cuboid.
 * @param {Object} [options] - options for construction
 * @param {Vector3} [options.center=[0,0,0]] - center of cube
 * @param {Vector3} [options.radius=[1,1,1]] - radius of cube, single scalar also possible
 * @returns {CSG} new 3D solid
 *
 * @example
 * let cube = CSG.cube({
 *   center: [5, 5, 5],
 *   radius: 5, // scalar radius
 * });
 */
const cube = function (options) {
  let c
  let r
  let corner1
  let corner2
  options = options || {}
  if (('corner1' in options) || ('corner2' in options)) {
    if (('center' in options) || ('radius' in options)) {
      throw new Error('cube: should either give a radius and center parameter, or a corner1 and corner2 parameter')
    }
    corner1 = parseOptionAs3DVector(options, 'corner1', [0, 0, 0])
    corner2 = parseOptionAs3DVector(options, 'corner2', [1, 1, 1])
    c = corner1.plus(corner2).times(0.5)
    r = corner2.minus(corner1).times(0.5)
  } else {
    c = parseOptionAs3DVector(options, 'center', [0, 0, 0])
    r = parseOptionAs3DVector(options, 'radius', [1, 1, 1])
  }
  r = r.abs() // negative radii make no sense
  let result = fromPolygons([
    [
            [0, 4, 6, 2],
            [-1, 0, 0]
    ],
    [
            [1, 3, 7, 5],
            [+1, 0, 0]
    ],
    [
            [0, 1, 5, 4],
            [0, -1, 0]
    ],
    [
            [2, 6, 7, 3],
            [0, +1, 0]
    ],
    [
            [0, 2, 3, 1],
            [0, 0, -1]
    ],
    [
            [4, 5, 7, 6],
            [0, 0, +1]
    ]
  ].map(function (info) {
    let vertices = info[0].map(function (i) {
      let pos = new Vector3(
                c.x + r.x * (2 * !!(i & 1) - 1), c.y + r.y * (2 * !!(i & 2) - 1), c.z + r.z * (2 * !!(i & 4) - 1))
      return new Vertex3(pos, new Vector3(info[1]))
    })
    return new Polygon3(vertices, null /* , plane */)
  }))
  result.properties.cube = new Properties()
  result.properties.cube.center = new Vector3(c)
    // add 6 connectors, at the centers of each face:
  result.properties.cube.facecenters = [
    new Connector(new Vector3([r.x, 0, 0]).plus(c), [1, 0, 0], [0, 0, 1]),
    new Connector(new Vector3([-r.x, 0, 0]).plus(c), [-1, 0, 0], [0, 0, 1]),
    new Connector(new Vector3([0, r.y, 0]).plus(c), [0, 1, 0], [0, 0, 1]),
    new Connector(new Vector3([0, -r.y, 0]).plus(c), [0, -1, 0], [0, 0, 1]),
    new Connector(new Vector3([0, 0, r.z]).plus(c), [0, 0, 1], [1, 0, 0]),
    new Connector(new Vector3([0, 0, -r.z]).plus(c), [0, 0, -1], [1, 0, 0])
  ]
  return result
}

/** Construct a solid sphere
 * @param {Object} [options] - options for construction
 * @param {Vector3} [options.center=[0,0,0]] - center of sphere
 * @param {Number} [options.radius=1] - radius of sphere
 * @param {Number} [options.resolution=defaultResolution3D] - number of polygons per 360 degree revolution
 * @param {Array} [options.axes] -  an array with 3 vectors for the x, y and z base vectors
 * @returns {CSG} new 3D solid
 *
 *
 * @example
 * let sphere = CSG.sphere({
 *   center: [0, 0, 0],
 *   radius: 2,
 *   resolution: 32,
 * });
*/
const sphere = function (options) {
  options = options || {}
  let center = parseOptionAs3DVector(options, 'center', [0, 0, 0])
  let radius = parseOptionAsFloat(options, 'radius', 1)
  let resolution = parseOptionAsInt(options, 'resolution', defaultResolution3D)
  let xvector, yvector, zvector
  if ('axes' in options) {
    xvector = options.axes[0].unit().times(radius)
    yvector = options.axes[1].unit().times(radius)
    zvector = options.axes[2].unit().times(radius)
  } else {
    xvector = new Vector3([1, 0, 0]).times(radius)
    yvector = new Vector3([0, -1, 0]).times(radius)
    zvector = new Vector3([0, 0, 1]).times(radius)
  }
  if (resolution < 4) resolution = 4
  let qresolution = Math.round(resolution / 4)
  let prevcylinderpoint
  let polygons = []
  function addVertex(vertices, center, radial) {
    vertices.push(new Vertex3(center.plus(radial), radial.unit()))
  }
  for (let slice1 = 0; slice1 <= resolution; slice1++) {
    let angle = Math.PI * 2.0 * slice1 / resolution
    let cylinderpoint = xvector.times(Math.cos(angle)).plus(yvector.times(Math.sin(angle)))
    if (slice1 > 0) {
            // cylinder vertices:
      let vertices = []
      let prevcospitch, prevsinpitch
      for (let slice2 = 0; slice2 <= qresolution; slice2++) {
        let pitch = 0.5 * Math.PI * slice2 / qresolution
        let cospitch = Math.cos(pitch)
        let sinpitch = Math.sin(pitch)
        if (slice2 > 0) {
          vertices = []
          addVertex(vertices, center, prevcylinderpoint.times(prevcospitch).minus(zvector.times(prevsinpitch)))
          addVertex(vertices, center, cylinderpoint.times(prevcospitch).minus(zvector.times(prevsinpitch)))
          if (slice2 < qresolution) {
            addVertex(vertices, center, cylinderpoint.times(cospitch).minus(zvector.times(sinpitch)))
          }
          addVertex(vertices, center, prevcylinderpoint.times(cospitch).minus(zvector.times(sinpitch)))
          polygons.push(new Polygon3(vertices))
          vertices = []
          addVertex(vertices, center, prevcylinderpoint.times(prevcospitch).plus(zvector.times(prevsinpitch)))
          addVertex(vertices, center, cylinderpoint.times(prevcospitch).plus(zvector.times(prevsinpitch)))
          if (slice2 < qresolution) {
            addVertex(vertices, center, cylinderpoint.times(cospitch).plus(zvector.times(sinpitch)))
          }
          addVertex(vertices, center, prevcylinderpoint.times(cospitch).plus(zvector.times(sinpitch)))
          vertices.reverse()
          polygons.push(new Polygon3(vertices))
        }
        prevcospitch = cospitch
        prevsinpitch = sinpitch
      }
    }
    prevcylinderpoint = cylinderpoint
  }
  let result = fromPolygons(polygons)
  result.properties.sphere = new Properties()
  result.properties.sphere.center = new Vector3(center)
  result.properties.sphere.facepoint = center.plus(xvector)
  return result
}

/** Construct a geodesic unit sphere (code from Brian Upton)
 * @param {Number} iterations=2 - number of subdivision iterations
 * @returns {CSG} new 3D solid
 *
 * @example
 * let sphere = CSG.unitSphere(3);
 */
function unitSphere(iterations = 2) {
  // 0:   20 triangles
  // 1:   80 triangles
  // 2:   320 triangles
  // 3:   1280 triangles
  // 4:   5120 triangles
  // 5:   20480 triangles
  // 6:   81920 triangles

  iterations = Math.min(iterations, 6);

  const t = (1 + Math.sqrt(5)) * 0.5;     // golden ratio
  const scale = Math.sqrt(t * t + 1);
  const a = 1 / scale;
  const b = t * a;

  const v0 = new Vector3([-a, b, 0]);
  const v1 = new Vector3([a, b, 0]);
  const v2 = new Vector3([-a, -b, 0]);
  const v3 = new Vector3([a, -b, 0]);

  const v4 = new Vector3([0, -a, b]);
  const v5 = new Vector3([0, a, b]);
  const v6 = new Vector3([0, -a, -b]);
  const v7 = new Vector3([0, a, -b]);

  const v8 = new Vector3([b, 0, -a]);
  const v9 = new Vector3([b, 0, a]);
  const v10 = new Vector3([-b, 0, -a]);
  const v11 = new Vector3([-b, 0, a]);

  let tris = [];

  // Top cap

  tris[0] = [v0, v11, v5];
  tris[1] = [v0, v5, v1];
  tris[2] = [v0, v1, v7];
  tris[3] = [v0, v7, v10];
  tris[4] = [v0, v10, v11];

  // Middle band

  tris[5] = [v1, v5, v9];
  tris[6] = [v5, v11, v4];
  tris[7] = [v11, v10, v2];
  tris[8] = [v10, v7, v6];
  tris[9] = [v7, v1, v8];

  tris[10] = [v4, v9, v5];
  tris[11] = [v2, v4, v11];
  tris[12] = [v6, v2, v10];
  tris[13] = [v8, v6, v7];
  tris[14] = [v9, v8, v1];

  // Bottom cap

  tris[15] = [v3, v9, v4];
  tris[16] = [v3, v4, v2];
  tris[17] = [v3, v2, v6];
  tris[18] = [v3, v6, v8];
  tris[19] = [v3, v8, v9];

  for (let i = 0; i < iterations; i++) {
    const sub = [];
    tris.forEach(tri => {
      const d = tri[0];
      const e = tri[1];
      const f = tri[2];
      let de = d.plus(e).unit();
      let df = d.plus(f).unit();
      let ef = e.plus(f).unit();
      sub.push([d, de, df]);
      sub.push([e, ef, de]);
      sub.push([f, df, ef]);
      sub.push([de, ef, df]);
    });
    tris = sub;
  }
  console.log(tris.length);

  const polygons = tris.map(vecs => new Polygon3(vecs.map(vec => {
    return new Vertex3(vec, vec);
  })));
  return fromPolygons(polygons);
}

/** Construct a trapezoidal prism, with a rectangular base centered at [0, 0, 0] and width on x,
 * depth on y, height in z.  Code from Aran Lunzer.
 *
 * @param {Object} [options] - options for construction
 * @param {Number} [options.height=1] - height of prism
 * @param {Number} [options.width1=1] - width (in x direction) at base
 * @param {Number} [options.depth1=1] - depth (in y direction) at base
 * @param {Number} [options.width2=width1] - width at top
 * @param {Number} [options.depth2=depth1] - depth at top
 * @returns {CSG} new 3D solid
 *
 * Example usage:
 *
 *     let trap = CSG.trapezoidPrism({
 *       height: 5,
 *       width1: 3,
 *       width2: 0,
 *       depth1: 4
 *     });
 */
function trapezoidPrism(options = {}) {
  const height = options.height || 1;
  const width1 = options.width1 || 1;
  const depth1 = options.depth1 || 1;
  const width2 = options.width2 || width1;
  const depth2 = options.depth2 || depth1;
  const w1h = width1 / 2, w2h = width2 / 2, d1h = depth1 / 2, d2h = depth2 / 2, h = height;
  const vertices = [
    [w1h, -d1h, 0],
    [w1h, d1h, 0],
    [-w1h, d1h, 0],
    [-w1h, -d1h, 0],
    [w2h, -d2h, h],
    [w2h, d2h, h],
    [-w2h, d2h, h],
    [-w2h, -d2h, h]
  ];
  const polygons = [
    [0, 3, 2, 1],
    [0, 1, 5, 4],
    [1, 2, 6, 5],
    [2, 3, 7, 6],
    [3, 0, 4, 7],
    [4, 5, 6, 7]
  ].map(seq => {
    const faceVects = seq.map(index => new Vector3(...vertices[index]));
    const normal = faceVects[1].minus(faceVects[0]).cross(faceVects[2].minus(faceVects[0])).unit();
    return new Polygon3(faceVects.map(vec => new Vertex3(vec, normal)));
  });
  const shape = fromPolygons(polygons);
  return shape;
}

/** Construct a solid cylinder.
 * @param {Object} [options] - options for construction
 * @param {Vector} [options.start=[0,-1,0]] - start point of cylinder
 * @param {Vector} [options.end=[0,1,0]] - end point of cylinder
 * @param {Number} [options.radius=1] - radius of cylinder, must be scalar
 * @param {Number} [options.resolution=defaultResolution3D] - number of polygons per 360 degree revolution
 * @returns {CSG} new 3D solid
 *
 * @example
 * let cylinder = CSG.cylinder({
 *   start: [0, -10, 0],
 *   end: [0, 10, 0],
 *   radius: 10,
 *   resolution: 16
 * });
 */
const cylinder = function (options) {
  let s = parseOptionAs3DVector(options, 'start', [0, -1, 0])
  let e = parseOptionAs3DVector(options, 'end', [0, 1, 0])
  let r = parseOptionAsFloat(options, 'radius', 1)
  let rEnd = parseOptionAsFloat(options, 'radiusEnd', r)
  let rStart = parseOptionAsFloat(options, 'radiusStart', r)
  let alpha = parseOptionAsFloat(options, 'sectorAngle', 360)
  alpha = alpha > 360 ? alpha % 360 : alpha

  if ((rEnd < 0) || (rStart < 0)) {
    throw new Error('Radius should be non-negative')
  }
  if ((rEnd === 0) && (rStart === 0)) {
    throw new Error('Either radiusStart or radiusEnd should be positive')
  }

  let slices = parseOptionAsInt(options, 'resolution', defaultResolution2D) // FIXME is this 3D?
  let ray = e.minus(s)
  let axisLength = ray.length()
  let axisZ = ray.unit() //, isY = (Math.abs(axisZ.y) > 0.5);
  let axisX = axisZ.randomNonParallelVector().unit()

    //  let axisX = new Vector3(isY, !isY, 0).cross(axisZ).unit();
  let axisY = axisX.cross(axisZ).unit()
  let start = new Vertex3(s, axisZ.negated())
  let end = new Vertex3(e, axisZ.clone())
  let coneTiltVector = axisZ.times((rStart-rEnd)/axisLength)

  let polygons = []

  function point (stack, slice, radius, normalBlend) {
    let angle = slice * Math.PI * alpha / 180
    let out = axisX.times(Math.cos(angle)).plus(axisY.times(Math.sin(angle)))
    let pos = s.plus(ray.times(stack)).plus(out.times(radius))
    let normal = normalBlend ? axisZ.times(normalBlend) : out.plus(coneTiltVector).unit()
    return new Vertex3(pos, normal)
  }

  if (alpha > 0) {
    for (let i = 0; i < slices; i++) {
      let t0 = i / slices
      let t1 = (i + 1) / slices
      if (rEnd === rStart) {
        polygons.push(new Polygon3([start, point(0, t0, rEnd, -1), point(0, t1, rEnd, -1)]))
        polygons.push(new Polygon3([point(0, t1, rEnd, 0), point(0, t0, rEnd, 0), point(1, t0, rEnd, 0), point(1, t1, rEnd, 0)]))
        polygons.push(new Polygon3([end, point(1, t1, rEnd, 1), point(1, t0, rEnd, 1)]))
      } else {
        if (rStart > 0) {
          polygons.push(new Polygon3([start, point(0, t0, rStart, -1), point(0, t1, rStart, -1)]))
          polygons.push(new Polygon3([point(0, t0, rStart, 0), point(1, t0, rEnd, 0), point(0, t1, rStart, 0)]))
        }
        if (rEnd > 0) {
          polygons.push(new Polygon3([end, point(1, t1, rEnd, 1), point(1, t0, rEnd, 1)]))
          polygons.push(new Polygon3([point(1, t0, rEnd, 0), point(1, t1, rEnd, 0), point(0, t1, rStart, 0)]))
        }
      }
    }
    // add the side planes for a reduced segment
    if (alpha < 360) {
      polygons.push(Polygon3.createFromVectors([s, e, point(0, 0, rStart, 0).pos]))
      polygons.push(Polygon3.createFromVectors([point(0, 0, rStart, 0).pos, e, point(1, 0, rEnd, 0).pos]))
      polygons.push(Polygon3.createFromVectors([s, point(0, 1, rStart, 0).pos, e]))
      polygons.push(Polygon3.createFromVectors([point(0, 1, rStart, 0).pos, point(1, 1, rEnd, 0).pos, e]))
    }
  }
  let result = fromPolygons(polygons)
  result.properties.cylinder = new Properties()
  result.properties.cylinder.start = new Connector(s, axisZ.negated(), axisX)
  result.properties.cylinder.end = new Connector(e, axisZ, axisX)
  let cylCenter = s.plus(ray.times(0.5))
  let fptVec = axisX.rotate(s, axisZ, -alpha / 2).times((rStart + rEnd) / 2)
  let fptVec90 = fptVec.cross(axisZ)
    // note this one is NOT a face normal for a cone. - It's horizontal from cyl perspective
  result.properties.cylinder.facepointH = new Connector(cylCenter.plus(fptVec), fptVec, axisZ)
  result.properties.cylinder.facepointH90 = new Connector(cylCenter.plus(fptVec90), fptVec90, axisZ)
  return result
}

/** Construct a cylinder with rounded ends.
 * @param {Object} [options] - options for construction
 * @param {Vector3} [options.start=[0,-1,0]] - start point of cylinder
 * @param {Vector3} [options.end=[0,1,0]] - end point of cylinder
 * @param {Number} [options.radius=1] - radius of rounded ends, must be scalar
 * @param {Vector3} [options.normal] - vector determining the starting angle for tesselation. Should be non-parallel to start.minus(end)
 * @param {Number} [options.resolution=defaultResolution3D] - number of polygons per 360 degree revolution
 * @returns {CSG} new 3D solid
 *
 * @example
 * let cylinder = CSG.roundedCylinder({
 *   start: [0, -10, 0],
 *   end: [0, 10, 0],
 *   radius: 2,
 *   resolution: 16
 * });
 */
const roundedCylinder = function (options) {
  let p1 = parseOptionAs3DVector(options, 'start', [0, -1, 0])
  let p2 = parseOptionAs3DVector(options, 'end', [0, 1, 0])
  let radius = parseOptionAsFloat(options, 'radius', 1)
  let direction = p2.minus(p1)
  let defaultnormal
  if (Math.abs(direction.x) > Math.abs(direction.y)) {
    defaultnormal = new Vector3(0, 1, 0)
  } else {
    defaultnormal = new Vector3(1, 0, 0)
  }
  let normal = parseOptionAs3DVector(options, 'normal', defaultnormal)
  let resolution = parseOptionAsInt(options, 'resolution', defaultResolution3D)
  if (resolution < 4) resolution = 4
  let polygons = []
  let qresolution = Math.floor(0.25 * resolution)
  let length = direction.length()
  if (length < EPS) {
    return sphere({
      center: p1,
      radius: radius,
      resolution: resolution
    })
  }
  let zvector = direction.unit().times(radius)
  let xvector = zvector.cross(normal).unit().times(radius)
  let yvector = xvector.cross(zvector).unit().times(radius)
  function addVertex(vertices, center, radial) {
    vertices.push(new Vertex3(center.plus(radial), radial.unit()))
  }

  let prevcylinderpoint
  for (let slice1 = 0; slice1 <= resolution; slice1++) {
    let angle = Math.PI * 2.0 * slice1 / resolution
    let cylinderpoint = xvector.times(Math.cos(angle)).plus(yvector.times(Math.sin(angle)))
    if (slice1 > 0) {
            // cylinder vertices:
      let vertices = []
      addVertex(vertices, p1, cylinderpoint)
      addVertex(vertices, p1, prevcylinderpoint)
      addVertex(vertices, p2, prevcylinderpoint)
      addVertex(vertices, p2, cylinderpoint)
      polygons.push(new Polygon3(vertices))
      let prevcospitch, prevsinpitch
      for (let slice2 = 0; slice2 <= qresolution; slice2++) {
        let pitch = 0.5 * Math.PI * slice2 / qresolution
                // let pitch = Math.asin(slice2/qresolution);
        let cospitch = Math.cos(pitch)
        let sinpitch = Math.sin(pitch)
        if (slice2 > 0) {
          vertices = []
          addVertex(vertices, p1, prevcylinderpoint.times(prevcospitch).minus(zvector.times(prevsinpitch)))
          addVertex(vertices, p1, cylinderpoint.times(prevcospitch).minus(zvector.times(prevsinpitch)))
          if (slice2 < qresolution) {
            addVertex(vertices, p1, cylinderpoint.times(cospitch).minus(zvector.times(sinpitch)))
          }
          addVertex(vertices, p1, prevcylinderpoint.times(cospitch).minus(zvector.times(sinpitch)))
          polygons.push(new Polygon3(vertices))
          vertices = []
          addVertex(vertices, p2, prevcylinderpoint.times(prevcospitch).plus(zvector.times(prevsinpitch)))
          addVertex(vertices, p2, cylinderpoint.times(prevcospitch).plus(zvector.times(prevsinpitch)))
          if (slice2 < qresolution) {
            addVertex(vertices, p2, cylinderpoint.times(cospitch).plus(zvector.times(sinpitch)))
          }
          addVertex(vertices, p2, prevcylinderpoint.times(cospitch).plus(zvector.times(sinpitch)))
          vertices.reverse()
          polygons.push(new Polygon3(vertices))
        }
        prevcospitch = cospitch
        prevsinpitch = sinpitch
      }
    }
    prevcylinderpoint = cylinderpoint
  }
  let result = fromPolygons(polygons)
  let ray = zvector.unit()
  let axisX = xvector.unit()
  result.properties.roundedCylinder = new Properties()
  result.properties.roundedCylinder.start = new Connector(p1, ray.negated(), axisX)
  result.properties.roundedCylinder.end = new Connector(p2, ray, axisX)
  result.properties.roundedCylinder.facepoint = p1.plus(xvector)
  return result
}

/** Construct an elliptic cylinder.
 * @param {Object} [options] - options for construction
 * @param {Vector3} [options.start=[0,-1,0]] - start point of cylinder
 * @param {Vector3} [options.end=[0,1,0]] - end point of cylinder
 * @param {Vector2D} [options.radius=[1,1]] - radius of rounded ends, must be two dimensional array
 * @param {Vector2D} [options.radiusStart=[1,1]] - OPTIONAL radius of rounded start, must be two dimensional array
 * @param {Vector2D} [options.radiusEnd=[1,1]] - OPTIONAL radius of rounded end, must be two dimensional array
 * @param {Number} [options.resolution=defaultResolution2D] - number of polygons per 360 degree revolution
 * @returns {CSG} new 3D solid
 *
 * @example
 *     let cylinder = CSG.cylinderElliptic({
 *       start: [0, -10, 0],
 *       end: [0, 10, 0],
 *       radiusStart: [10,5],
 *       radiusEnd: [8,3],
 *       resolution: 16
 *     });
 */

const cylinderElliptic = function (options) {
  let s = parseOptionAs3DVector(options, 'start', [0, -1, 0])
  let e = parseOptionAs3DVector(options, 'end', [0, 1, 0])
  let r = parseOptionAs2DVector(options, 'radius', [1, 1])
  let rEnd = parseOptionAs2DVector(options, 'radiusEnd', r)
  let rStart = parseOptionAs2DVector(options, 'radiusStart', r)

  if ((rEnd._x < 0) || (rStart._x < 0) || (rEnd._y < 0) || (rStart._y < 0)) {
    throw new Error('Radius should be non-negative')
  }
  if ((rEnd._x === 0 || rEnd._y === 0) && (rStart._x === 0 || rStart._y === 0)) {
    throw new Error('Either radiusStart or radiusEnd should be positive')
  }

  let slices = parseOptionAsInt(options, 'resolution', defaultResolution2D) // FIXME is this correct?
  let ray = e.minus(s)
  let axisLength = ray.length()
  let axisZ = ray.unit() //, isY = (Math.abs(axisZ.y) > 0.5);
  let axisX = axisZ.randomNonParallelVector().unit()

    //  let axisX = new Vector3(isY, !isY, 0).cross(axisZ).unit();
  let axisY = axisX.cross(axisZ).unit()
  let start = new Vertex3(s, axisZ.negated())
  let end = new Vertex3(e, axisZ.clone())
  let polygons = []

  function point (stack, slice, endNormal) {
    let angle = slice * Math.PI * 2, cos = Math.cos(angle), sin = Math.sin(angle)
    let radii = [{ x: rStart._x * cos, y: rStart._y * sin }, { x: rEnd._x * cos, y: rEnd._y * sin }]
    let radiusHere = radii[stack]
    let out = axisX.times(radiusHere.x).plus(axisY.times(radiusHere.y))
    let pos = s.plus(ray.times(stack)).plus(out)
    let normal
    if (endNormal) normal = axisZ.times(endNormal)
    else {
      let rMags = radii.map(xy => Math.sqrt(xy.x*xy.x + xy.y*xy.y))
      let tiltVector = axisZ.times((rMags[0] - rMags[1])/axisLength)
      normal = out.unit().plus(tiltVector).unit()
    }
    return new Vertex3(pos, normal)
  }
  for (let i = 0; i < slices; i++) {
    let t0 = i / slices
    let t1 = (i + 1) / slices

    if (rEnd._x === rStart._x && rEnd._y === rStart._y) {
      polygons.push(new Polygon3([start, point(0, t0, -1), point(0, t1, -1)]))
      polygons.push(new Polygon3([point(0, t1, 0), point(0, t0, 0), point(1, t0, 0), point(1, t1, 0)]))
      polygons.push(new Polygon3([end, point(1, t1, 1), point(1, t0, 1)]))
    } else {
      if (rStart._x > 0) {
        polygons.push(new Polygon3([start, point(0, t0, -1), point(0, t1, -1)]))
        polygons.push(new Polygon3([point(0, t0, 0), point(1, t0, 0), point(0, t1, 0)]))
      }
      if (rEnd._x > 0) {
        polygons.push(new Polygon3([end, point(1, t1, 1), point(1, t0, 1)]))
        polygons.push(new Polygon3([point(1, t0, 0), point(1, t1, 0), point(0, t1, 0)]))
      }
    }
  }
  let result = fromPolygons(polygons)
  result.properties.cylinder = new Properties()
  result.properties.cylinder.start = new Connector(s, axisZ.negated(), axisX)
  result.properties.cylinder.end = new Connector(e, axisZ, axisX)
  result.properties.cylinder.facepoint = s.plus(axisX.times(rStart))
  return result
}

/** Construct an axis-aligned solid rounded cuboid.
 * @param {Object} [options] - options for construction
 * @param {Vector3} [options.center=[0,0,0]] - center of rounded cube
 * @param {Vector3} [options.radius=[1,1,1]] - radius of rounded cube, single scalar is possible
 * @param {Number} [options.roundradius=0.2] - radius of rounded edges
 * @param {Number} [options.resolution=defaultResolution3D] - number of polygons per 360 degree revolution
 * @returns {CSG} new 3D solid
 *
 * @example
 * let cube = CSG.roundedCube({
 *   center: [2, 0, 2],
 *   radius: 15,
 *   roundradius: 2,
 *   resolution: 36,
 * });
 */
const roundedCube = function (options) {
  let minRR = 1e-2 // minroundradius 1e-3 gives rounding errors already
  let center
  let cuberadius
  let corner1
  let corner2
  options = options || {}
  if (('corner1' in options) || ('corner2' in options)) {
    if (('center' in options) || ('radius' in options)) {
      throw new Error('roundedCube: should either give a radius and center parameter, or a corner1 and corner2 parameter')
    }
    corner1 = parseOptionAs3DVector(options, 'corner1', [0, 0, 0])
    corner2 = parseOptionAs3DVector(options, 'corner2', [1, 1, 1])
    center = corner1.plus(corner2).times(0.5)
    cuberadius = corner2.minus(corner1).times(0.5)
  } else {
    center = parseOptionAs3DVector(options, 'center', [0, 0, 0])
    cuberadius = parseOptionAs3DVector(options, 'radius', [1, 1, 1])
  }
  cuberadius = cuberadius.abs() // negative radii make no sense
  let resolution = parseOptionAsInt(options, 'resolution', defaultResolution3D)
  if (resolution < 4) resolution = 4
  if (resolution % 2 === 1 && resolution < 8) resolution = 8 // avoid ugly
  let roundradius = parseOptionAs3DVector(options, 'roundradius', [0.2, 0.2, 0.2])
    // slight hack for now - total radius stays ok
  roundradius = Vector3.Create(Math.max(roundradius.x, minRR), Math.max(roundradius.y, minRR), Math.max(roundradius.z, minRR))
  let innerradius = cuberadius.minus(roundradius)
  if (innerradius.x < 0 || innerradius.y < 0 || innerradius.z < 0) {
    throw new Error('roundradius <= radius!')
  }
  let res = sphere({ radius: 1, resolution })
  res = res.scale(roundradius)
  if (innerradius.x > EPS) res = res.simplifiedStretchAtPlane([1, 0, 0], [0, 0, 0], 2 * innerradius.x)
  if (innerradius.y > EPS) res = res.simplifiedStretchAtPlane([0, 1, 0], [0, 0, 0], 2 * innerradius.y)
  if (innerradius.z > EPS) res = res.simplifiedStretchAtPlane([0, 0, 1], [0, 0, 0], 2 * innerradius.z)
  res = res.translate([-innerradius.x + center.x, -innerradius.y + center.y, -innerradius.z + center.z])
  res = res.reTesselated()
  res.properties.roundedCube = new Properties()
  res.properties.roundedCube.center = new Vector3(center) // ael - used to be a Vertex, but not clear it needs to be
  res.properties.roundedCube.facecenters = [
    new Connector(new Vector3([cuberadius.x, 0, 0]).plus(center), [1, 0, 0], [0, 0, 1]),
    new Connector(new Vector3([-cuberadius.x, 0, 0]).plus(center), [-1, 0, 0], [0, 0, 1]),
    new Connector(new Vector3([0, cuberadius.y, 0]).plus(center), [0, 1, 0], [0, 0, 1]),
    new Connector(new Vector3([0, -cuberadius.y, 0]).plus(center), [0, -1, 0], [0, 0, 1]),
    new Connector(new Vector3([0, 0, cuberadius.z]).plus(center), [0, 0, 1], [1, 0, 0]),
    new Connector(new Vector3([0, 0, -cuberadius.z]).plus(center), [0, 0, -1], [1, 0, 0])
  ]
  return res
}

/** Create a polyhedron using Openscad style arguments.
 * Define face vertices clockwise looking from outside.
 * @param {Object} [options] - options for construction
 * @returns {CSG} new 3D solid
 */
const polyhedron = function (options) {
  options = options || {}
  if (('points' in options) !== ('faces' in options)) {
    throw new Error("polyhedron needs 'points' and 'faces' arrays")
  }
  let vertices = parseOptionAs3DVectorList(options, 'points', [
            [1, 1, 0],
            [1, -1, 0],
            [-1, -1, 0],
            [-1, 1, 0],
            [0, 0, 1]
  ])
        .map(function (pt) {
          return new Vertex3(pt)
        })
  let faces = parseOption(options, 'faces', [
            [0, 1, 4],
            [1, 2, 4],
            [2, 3, 4],
            [3, 0, 4],
            [1, 0, 3],
            [2, 1, 3]
  ])
    // Openscad convention defines inward normals - so we have to invert here
  faces.forEach(function (face) {
    face.reverse()
  })
  let polygons = faces.map(function (face) {
    return new Polygon3(face.map(function (idx) {
      return vertices[idx]
    }))
  })

    // TODO: facecenters as connectors? probably overkill. Maybe centroid
    // the re-tesselation here happens because it's so easy for a user to
    // create parametrized polyhedrons that end up with 1-2 dimensional polygons.
    // These will create infinite loops at CSG.Tree()
  return fromPolygons(polygons).reTesselated()
}

module.exports = {
  cube,
  sphere,
  unitSphere,
  trapezoidPrism,
  roundedCube,
  cylinder,
  roundedCylinder,
  cylinderElliptic,
  polyhedron
}
