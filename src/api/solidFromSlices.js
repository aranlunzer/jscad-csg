/* eslint-disable semi */
const Polygon = require('../core/math/Polygon3')
const Vertex = require('../core/math/Vertex3')
const {fromPolygons} = require('../core/CSGFactories')
const {fnSortByIndex} = require('../core/utils')

// FIXME: WHY is this for 3D polygons and not for 2D shapes ?
/**
 * Creates solid from slices (Polygon) by generating walls
 * @param {Object} options Solid generating options
 *  - numslices {Number} Number of slices to be generated
 *  - callback(t, slice) {Function} Callback function generating slices.
 *          arguments: t = [0..1], slice = [0..numslices - 1]
 *          return: Polygon or null to skip
 *  - loop {Boolean} no flats, only walls, it's used to generate solids like a tor
 *  - cornerSmoothing {Boolean} (ael added) - average and share vertex normals on corners
 *  - maxSmoothableAngle {Number} (ael added) - inter-face angles (on a slice) sharper than this many degrees won't be smoothed
 */
const solidFromSlices = function (polygon, options) {
  let polygons = []
  let csg = null
  let prev = null
  let bottom = null
  let top = null
  let numSlices = 2
  let bLoop = false
  let fnCallback
  let flipped = null
  let cornerSmoothing = false
  let maxSmoothableAngle = 30

  if (options) {
    bLoop = Boolean(options['loop'])

    if (options.numslices) { numSlices = options.numslices }

    if (options.callback) {
      fnCallback = options.callback
    }

    if (options.cornerSmoothing) cornerSmoothing = options.cornerSmoothing

    if (options.maxSmoothableAngle) maxSmoothableAngle = options.maxSmoothableAngle
  }
  if (!fnCallback) {
    let square = Polygon.createFromPoints([
                  [0, 0, 0],
                  [1, 0, 0],
                  [1, 1, 0],
                  [0, 1, 0]
    ])
    fnCallback = function (t, slice) {
      return t === 0 || t === 1 ? square.translate([0, 0, t]) : null
    }
  }
  for (let i = 0, iMax = numSlices - 1; i <= iMax; i++) {
    csg = fnCallback.call(polygon, i / iMax, i)
    if (csg) {
      if (!(csg instanceof Polygon)) {
        throw new Error('Polygon.solidFromSlices callback error: Polygon expected')
      }
      csg.checkIfConvex()

      if (prev) { // generate walls
        if (flipped === null) { // not generated yet
          flipped = prev.plane.signedDistanceToPoint(csg.vertices[0].pos) < 0
        }
        _addWalls(polygons, prev, csg, flipped, cornerSmoothing, maxSmoothableAngle)
      } else { // the first - will be a bottom
        bottom = csg
      }
      prev = csg
    } // callback can return null to skip that slice
  }
  top = csg

  if (bLoop) {
    let bSameTopBottom = bottom.vertices.length === top.vertices.length &&
                  bottom.vertices.every(function (v, index) {
                    return v.pos.equals(top.vertices[index].pos)
                  })
    // if top and bottom are not the same -
    // generate walls between them
    if (!bSameTopBottom) {
      _addWalls(polygons, top, bottom, flipped, cornerSmoothing, maxSmoothableAngle)
    } // else - already generated
  } else {
    // save top and bottom
    // TODO: flip if necessary
    polygons.unshift(flipped ? bottom : bottom.flipped())
    polygons.push(flipped ? top.flipped() : top)
  }
  return fromPolygons(polygons)
}

/**
 * @param walls Array of wall polygons
 * @param bottom Bottom polygon
 * @param top Top polygon
 */
const _addWalls = function (walls, bottom, top, bFlipped, cornerSmoothing, maxSmoothableAngle) {
  let copyVertex = (vert, topBot) => {
    let newV = Vertex.fromObject(vert);
    newV._interPlaneVertices = [];
    newV._cornerVertices = [];
    newV.topBot = topBot;
    return newV;
  }
  let bottomPoints = bottom.vertices.map(vert => copyVertex(vert, "bot"))
  let topPoints = top.vertices.map(vert => copyVertex(vert, "top"))

  let color = top.shared || null

        // check if bottom perimeter is closed
  if (!bottomPoints[0].pos.equals(bottomPoints[bottomPoints.length - 1].pos)) {
    bottomPoints.push(bottomPoints[0])
  }

        // check if top perimeter is closed
  if (!topPoints[0].pos.equals(topPoints[topPoints.length - 1].pos)) {
    topPoints.push(topPoints[0])
  }
  if (bFlipped) {
    bottomPoints = bottomPoints.reverse()
    topPoints = topPoints.reverse()
  }

  let iTopLen = topPoints.length - 1
  let iBotLen = bottomPoints.length - 1
  let iExtra = iTopLen - iBotLen// how many extra triangles we need
  let bMoreTops = iExtra > 0
  let bMoreBottoms = iExtra < 0

  let aMin = [] // indexes to start extra triangles (polygon with minimal square)
        // init - we need exactly /iExtra/ small triangles
  for (let i = Math.abs(iExtra); i > 0; i--) {
    aMin.push({
      len: Infinity,
      index: -1
    })
  }

  let len
  if (bMoreBottoms) {
    for (let i = 0; i < iBotLen; i++) {
      len = bottomPoints[i].pos.distanceToSquared(bottomPoints[i + 1].pos)
                // find the element to replace
      for (let j = aMin.length - 1; j >= 0; j--) {
        if (aMin[j].len > len) {
          aMin[j].len = len
          aMin.index = j
          break
        }
      } // for
    }
  } else if (bMoreTops) {
    for (let i = 0; i < iTopLen; i++) {
      len = topPoints[i].pos.distanceToSquared(topPoints[i + 1].pos)
                // find the element to replace
      for (let j = aMin.length - 1; j >= 0; j--) {
        if (aMin[j].len > len) {
          aMin[j].len = len
          aMin.index = j
          break
        }
      } // for
    }
  } // if
  // sort by index
  aMin.sort(fnSortByIndex)
  let getTriangle = function(vertices, colorShared, commonTopBot) {
    // make a triangle using independent vertices.
    let triangle = Polygon.createFromVectors(vertices.map(vert => vert.pos), colorShared)

    if (!cornerSmoothing) return triangle  // end of story

    // if this shape is subject to smoothing, then on the edge (top or bottom) that
    // supplies two vertices for this triangle, record in each supplied vertex the
    // new vertex that shares its position
    vertices.forEach((vert, i) => {
      let newVert = triangle.vertices[i]
      if (vert.topBot===commonTopBot) vert._interPlaneVertices.push(newVert)
      else vert._cornerVertices.push(newVert)
    })
    return triangle
    // return bFlipped ? triangle.flipped() : triangle;
  }

  let bpoint = bottomPoints[0]
  let tpoint = topPoints[0]
  let secondPoint
  let nBotFacet
  let nTopFacet // length of triangle facet side
  for (let iB = 0, iT = 0, iMax = iTopLen + iBotLen; iB + iT < iMax;) {
    if (aMin.length) {
      if (bMoreTops && iT === aMin[0].index) { // one vertex is on the bottom, 2 - on the top
        secondPoint = topPoints[++iT]
                    // console.log('<<< extra top: ' + secondPoint + ', ' + tpoint + ', bottom: ' + bpoint);
        walls.push(getTriangle(
                        [secondPoint, tpoint, bpoint], color, "top"
                    ))
        tpoint = secondPoint
        aMin.shift()
        continue
      } else if (bMoreBottoms && iB === aMin[0].index) {
        secondPoint = bottomPoints[++iB]
        walls.push(getTriangle(
                        [tpoint, bpoint, secondPoint], color, "bot"
                    ))
        bpoint = secondPoint
        aMin.shift()
        continue
      }
    }
            // choose the shortest path
    if (iB < iBotLen) { // one vertex is on the top, 2 - on the bottom
      nBotFacet = tpoint.pos.distanceToSquared(bottomPoints[iB + 1].pos)
    } else {
      nBotFacet = Infinity
    }
    if (iT < iTopLen) { // one vertex is on the bottom, 2 - on the top
      nTopFacet = bpoint.pos.distanceToSquared(topPoints[iT + 1].pos)
    } else {
      nTopFacet = Infinity
    }
    if (nBotFacet <= nTopFacet) {
      secondPoint = bottomPoints[++iB]
      walls.push(getTriangle(
                    [tpoint, bpoint, secondPoint], color, "bot"
                ))
      bpoint = secondPoint
    } else if (iT < iTopLen) { // nTopFacet < Infinity
      secondPoint = topPoints[++iT]
                // console.log('<<< top: ' + secondPoint + ', ' + tpoint + ', bottom: ' + bpoint);
      walls.push(getTriangle(
                    [secondPoint, tpoint, bpoint], color, "top"
                ))
      tpoint = secondPoint
    }
  }

  if (cornerSmoothing) {
    // ael - now go around all top and bottom vertices and average the normals of
    // those shared between walls whose angle is less acute than a threshold
    let dotThreshold = Math.cos(maxSmoothableAngle*Math.PI/180); // dot less than this is more acute.
    let allPoints = bottomPoints.concat(topPoints);
    allPoints.forEach(vert => {
      let derivedVertices = vert._interPlaneVertices;
      if (derivedVertices.length===2) {
        let [n0, n1] = derivedVertices.map(dv => dv.normal)
        if (n0.dot(n1) >= dotThreshold) {
          let newNormal = n0.plus(n1).unit()
          derivedVertices.forEach(dv => dv.normal = newNormal.clone())
          vert._cornerVertices.forEach(dv => dv.normal = newNormal.clone())
        }
      } else console.warn(`unexpected shape: ${derivedVertices.length} adjacent triangles`);
    })
    allPoints.forEach(vert => delete vert._interPlaneVertices);
  }
}

module.exports = solidFromSlices
