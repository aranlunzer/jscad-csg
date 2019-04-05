const Vector3D = require('./Vector3')
const {getTag} = require('../constants')

// # class Vertex
// Represents a vertex of a polygon. Use your own vertex class instead of this
// one to provide additional features like texture coordinates and vertex
// colors. Custom vertex classes need to provide a `pos` property
// `flipped()`, and `interpolate()` methods that behave analogous to the ones
// FIXME: And a lot MORE (see plane.fromVector3Ds for ex) ! This is fragile code
// defined by `Vertex`.
const Vertex = function (pos, normal) { if (!normal) debugger;
  this.pos = pos
  this.normal = normal
}

// create from an untyped object with identical property names:
Vertex.fromObject = function (obj) {
  var pos = new Vector3D(obj.pos)
  var normal = new Vector3D(obj.normal)
  return new Vertex(pos, normal)
}

Vertex.prototype = {
    // Return a vertex with all orientation-specific data (e.g. vertex normal) flipped. Called when the
    // orientation of a polygon is flipped.
  flipped: function () {
    return new Vertex(this.pos.clone(), this.normal.negated())
  },

  getTag: function () {
    var result = this.tag
    if (!result) {
      result = getTag()
      this.tag = result
    }
    return result
  },

    // Create a new vertex between this vertex and `other` by linearly
    // interpolating all properties using a parameter of `t`. Subclasses should
    // override this to interpolate additional properties.
  interpolate: function (other, t) {
    return new Vertex(
      this.pos.lerp(other.pos, t),
      this.normal.lerp(other.normal, t)
    );
  },

    // Affine transformation of vertex. Returns a new Vertex
  transform: function (matrix4x4) {
    var newpos = this.pos.multiply4x4(matrix4x4)
    var nMat = matrix4x4.inverseTranspose().reducedTo3x3()
    var newnorm = this.normal.multiply4x4(nMat).unit()
    return new Vertex(newpos, newnorm)
  },

  toString: function () {
    return this.pos.toString()
  }
}

module.exports = Vertex
