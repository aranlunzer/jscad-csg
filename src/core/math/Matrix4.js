const Vector3D = require('./Vector3')
const Vector2D = require('./Vector2')
const OrthoNormalBasis = require('./OrthoNormalBasis')
const Plane = require('./Plane')

// # class Matrix4x4:
// Represents a 4x4 matrix. Elements are specified in row order
const Matrix4x4 = function (elements) {
  if (arguments.length >= 1) {
    this.elements = elements
  } else {
        // if no arguments passed: create unity matrix
    this.elements = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
  }
}

Matrix4x4.prototype = {
  plus: function (m) {
    var r = []
    for (var i = 0; i < 16; i++) {
      r[i] = this.elements[i] + m.elements[i]
    }
    return new Matrix4x4(r)
  },

  minus: function (m) {
    var r = []
    for (var i = 0; i < 16; i++) {
      r[i] = this.elements[i] - m.elements[i]
    }
    return new Matrix4x4(r)
  },

    // right multiply by another 4x4 matrix:
  multiply: function (m) {
        // cache elements in local variables, for speedup:
    var this0 = this.elements[0]
    var this1 = this.elements[1]
    var this2 = this.elements[2]
    var this3 = this.elements[3]
    var this4 = this.elements[4]
    var this5 = this.elements[5]
    var this6 = this.elements[6]
    var this7 = this.elements[7]
    var this8 = this.elements[8]
    var this9 = this.elements[9]
    var this10 = this.elements[10]
    var this11 = this.elements[11]
    var this12 = this.elements[12]
    var this13 = this.elements[13]
    var this14 = this.elements[14]
    var this15 = this.elements[15]
    var m0 = m.elements[0]
    var m1 = m.elements[1]
    var m2 = m.elements[2]
    var m3 = m.elements[3]
    var m4 = m.elements[4]
    var m5 = m.elements[5]
    var m6 = m.elements[6]
    var m7 = m.elements[7]
    var m8 = m.elements[8]
    var m9 = m.elements[9]
    var m10 = m.elements[10]
    var m11 = m.elements[11]
    var m12 = m.elements[12]
    var m13 = m.elements[13]
    var m14 = m.elements[14]
    var m15 = m.elements[15]

    var result = []
    result[0] = this0 * m0 + this1 * m4 + this2 * m8 + this3 * m12
    result[1] = this0 * m1 + this1 * m5 + this2 * m9 + this3 * m13
    result[2] = this0 * m2 + this1 * m6 + this2 * m10 + this3 * m14
    result[3] = this0 * m3 + this1 * m7 + this2 * m11 + this3 * m15
    result[4] = this4 * m0 + this5 * m4 + this6 * m8 + this7 * m12
    result[5] = this4 * m1 + this5 * m5 + this6 * m9 + this7 * m13
    result[6] = this4 * m2 + this5 * m6 + this6 * m10 + this7 * m14
    result[7] = this4 * m3 + this5 * m7 + this6 * m11 + this7 * m15
    result[8] = this8 * m0 + this9 * m4 + this10 * m8 + this11 * m12
    result[9] = this8 * m1 + this9 * m5 + this10 * m9 + this11 * m13
    result[10] = this8 * m2 + this9 * m6 + this10 * m10 + this11 * m14
    result[11] = this8 * m3 + this9 * m7 + this10 * m11 + this11 * m15
    result[12] = this12 * m0 + this13 * m4 + this14 * m8 + this15 * m12
    result[13] = this12 * m1 + this13 * m5 + this14 * m9 + this15 * m13
    result[14] = this12 * m2 + this13 * m6 + this14 * m10 + this15 * m14
    result[15] = this12 * m3 + this13 * m7 + this14 * m11 + this15 * m15
    return new Matrix4x4(result)
  },

  clone: function () {
    var elements = this.elements.map(function (p) {
      return p
    })
    return new Matrix4x4(elements)
  },

    // Right multiply the matrix by a Vector3D (interpreted as 3 row, 1 column)
    // (result = M*v)
    // Fourth element is taken as 1
  rightMultiply1x3Vector: function (v) {
    var v0 = v._x
    var v1 = v._y
    var v2 = v._z
    var v3 = 1
    var x = v0 * this.elements[0] + v1 * this.elements[1] + v2 * this.elements[2] + v3 * this.elements[3]
    var y = v0 * this.elements[4] + v1 * this.elements[5] + v2 * this.elements[6] + v3 * this.elements[7]
    var z = v0 * this.elements[8] + v1 * this.elements[9] + v2 * this.elements[10] + v3 * this.elements[11]
    var w = v0 * this.elements[12] + v1 * this.elements[13] + v2 * this.elements[14] + v3 * this.elements[15]
        // scale such that fourth element becomes 1:
    if (w !== 1) {
      var invw = 1.0 / w
      x *= invw
      y *= invw
      z *= invw
    }
    return new Vector3D(x, y, z)
  },

    // Multiply a Vector3D (interpreted as 3 column, 1 row) by this matrix
    // (result = v*M)
    // Fourth element is taken as 1
  leftMultiply1x3Vector: function (v) {
    var v0 = v._x
    var v1 = v._y
    var v2 = v._z
    var v3 = 1
    var x = v0 * this.elements[0] + v1 * this.elements[4] + v2 * this.elements[8] + v3 * this.elements[12]
    var y = v0 * this.elements[1] + v1 * this.elements[5] + v2 * this.elements[9] + v3 * this.elements[13]
    var z = v0 * this.elements[2] + v1 * this.elements[6] + v2 * this.elements[10] + v3 * this.elements[14]
    var w = v0 * this.elements[3] + v1 * this.elements[7] + v2 * this.elements[11] + v3 * this.elements[15]
        // scale such that fourth element becomes 1:
    if (w !== 1) {
      var invw = 1.0 / w
      x *= invw
      y *= invw
      z *= invw
    }
    return new Vector3D(x, y, z)
  },

    // Right multiply the matrix by a Vector2D (interpreted as 2 row, 1 column)
    // (result = M*v)
    // Fourth element is taken as 1
  rightMultiply1x2Vector: function (v) {
    var v0 = v.x
    var v1 = v.y
    var v2 = 0
    var v3 = 1
    var x = v0 * this.elements[0] + v1 * this.elements[1] + v2 * this.elements[2] + v3 * this.elements[3]
    var y = v0 * this.elements[4] + v1 * this.elements[5] + v2 * this.elements[6] + v3 * this.elements[7]
    var z = v0 * this.elements[8] + v1 * this.elements[9] + v2 * this.elements[10] + v3 * this.elements[11]
    var w = v0 * this.elements[12] + v1 * this.elements[13] + v2 * this.elements[14] + v3 * this.elements[15]
        // scale such that fourth element becomes 1:
    if (w !== 1) {
      var invw = 1.0 / w
      x *= invw
      y *= invw
      z *= invw
    }
    return new Vector2D(x, y)
  },

    // Multiply a Vector2D (interpreted as 2 column, 1 row) by this matrix
    // (result = v*M)
    // Fourth element is taken as 1
  leftMultiply1x2Vector: function (v) {
    var v0 = v.x
    var v1 = v.y
    var v2 = 0
    var v3 = 1
    var x = v0 * this.elements[0] + v1 * this.elements[4] + v2 * this.elements[8] + v3 * this.elements[12]
    var y = v0 * this.elements[1] + v1 * this.elements[5] + v2 * this.elements[9] + v3 * this.elements[13]
    var z = v0 * this.elements[2] + v1 * this.elements[6] + v2 * this.elements[10] + v3 * this.elements[14]
    var w = v0 * this.elements[3] + v1 * this.elements[7] + v2 * this.elements[11] + v3 * this.elements[15]
        // scale such that fourth element becomes 1:
    if (w !== 1) {
      var invw = 1.0 / w
      x *= invw
      y *= invw
      z *= invw
    }
    return new Vector2D(x, y)
  },

    // determine whether this matrix is a mirroring transformation
  isMirroring: function () {
    var u = new Vector3D(this.elements[0], this.elements[4], this.elements[8])
    var v = new Vector3D(this.elements[1], this.elements[5], this.elements[9])
    var w = new Vector3D(this.elements[2], this.elements[6], this.elements[10])

        // for a true orthogonal, non-mirrored base, u.cross(v) == w
        // If they have an opposite direction then we are mirroring
    var mirrorvalue = u.cross(v).dot(w)
    var ismirror = (mirrorvalue < 0)
    return ismirror
  },

  // ael - copied from glmatrix.net's mat4, which is column-major
  inverseTranspose: function() {
    let a = Matrix4x4.transposeElementArray(this.elements);
    let a00 = a[0], a01 = a[1], a02 = a[2], a03 = a[3];
    let a10 = a[4], a11 = a[5], a12 = a[6], a13 = a[7];
    let a20 = a[8], a21 = a[9], a22 = a[10], a23 = a[11];
    let a30 = a[12], a31 = a[13], a32 = a[14], a33 = a[15];
    let b00 = a00 * a11 - a01 * a10;
    let b01 = a00 * a12 - a02 * a10;
    let b02 = a00 * a13 - a03 * a10;
    let b03 = a01 * a12 - a02 * a11;
    let b04 = a01 * a13 - a03 * a11;
    let b05 = a02 * a13 - a03 * a12;
    let b06 = a20 * a31 - a21 * a30;
    let b07 = a20 * a32 - a22 * a30;
    let b08 = a20 * a33 - a23 * a30;
    let b09 = a21 * a32 - a22 * a31;
    let b10 = a21 * a33 - a23 * a31;
    let b11 = a22 * a33 - a23 * a32;
    // Calculate the determinant
    let det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
    if (!det) {
      return null;
    }
    det = 1.0 / det;
    let out = new Array(16);
    out[0] = (a11 * b11 - a12 * b10 + a13 * b09) * det;
    out[1] = (a02 * b10 - a01 * b11 - a03 * b09) * det;
    out[2] = (a31 * b05 - a32 * b04 + a33 * b03) * det;
    out[3] = (a22 * b04 - a21 * b05 - a23 * b03) * det;
    out[4] = (a12 * b08 - a10 * b11 - a13 * b07) * det;
    out[5] = (a00 * b11 - a02 * b08 + a03 * b07) * det;
    out[6] = (a32 * b02 - a30 * b05 - a33 * b01) * det;
    out[7] = (a20 * b05 - a22 * b02 + a23 * b01) * det;
    out[8] = (a10 * b10 - a11 * b08 + a13 * b06) * det;
    out[9] = (a01 * b08 - a00 * b10 - a03 * b06) * det;
    out[10] = (a30 * b04 - a31 * b02 + a33 * b00) * det;
    out[11] = (a21 * b02 - a20 * b04 - a23 * b00) * det;
    out[12] = (a11 * b07 - a10 * b09 - a12 * b06) * det;
    out[13] = (a00 * b09 - a01 * b07 + a02 * b06) * det;
    out[14] = (a31 * b01 - a30 * b03 - a32 * b00) * det;
    out[15] = (a20 * b03 - a21 * b01 + a22 * b00) * det;
    return new Matrix4x4(out);
  },

  transposed() {
    return new Matrix4x4(Matrix4x4.transposeElementArray(this.elements))
  },

  reducedTo3x3() {
    let e = this.elements.slice();
    e[3] = e[7] = e[11] = e[12] = e[13] = e[14] = 0;
    e[15] = 1;
    return new Matrix4x4(e);
  }
}

Matrix4x4.transposeElementArray = function(a) {
  let out = new Array(15);
  out[0] = a[0];
  out[1] = a[4];
  out[2] = a[8];
  out[3] = a[12];
  out[4] = a[1];
  out[5] = a[5];
  out[6] = a[9];
  out[7] = a[13];
  out[8] = a[2];
  out[9] = a[6];
  out[10] = a[10];
  out[11] = a[14];
  out[12] = a[3];
  out[13] = a[7];
  out[14] = a[11];
  out[15] = a[15];
  return out;
}

// return the unity matrix
Matrix4x4.unity = function () {
  return new Matrix4x4()
}

// Create a rotation matrix for rotating around the x axis
Matrix4x4.rotationX = function (degrees) {
  var radians = degrees * Math.PI * (1.0 / 180.0)
  var cos = Math.cos(radians)
  var sin = Math.sin(radians)
  var els = [
    1, 0, 0, 0, 0, cos, sin, 0, 0, -sin, cos, 0, 0, 0, 0, 1
  ]
  return new Matrix4x4(els)
}

// Create a rotation matrix for rotating around the y axis
Matrix4x4.rotationY = function (degrees) {
  var radians = degrees * Math.PI * (1.0 / 180.0)
  var cos = Math.cos(radians)
  var sin = Math.sin(radians)
  var els = [
    cos, 0, -sin, 0, 0, 1, 0, 0, sin, 0, cos, 0, 0, 0, 0, 1
  ]
  return new Matrix4x4(els)
}

// Create a rotation matrix for rotating around the z axis
Matrix4x4.rotationZ = function (degrees) {
  var radians = degrees * Math.PI * (1.0 / 180.0)
  var cos = Math.cos(radians)
  var sin = Math.sin(radians)
  var els = [
    cos, sin, 0, 0, -sin, cos, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  ]
  return new Matrix4x4(els)
}

// Matrix for rotation about arbitrary point and axis
Matrix4x4.rotation = function (rotationCenter, rotationAxis, degrees) {
  rotationCenter = new Vector3D(rotationCenter)
  rotationAxis = new Vector3D(rotationAxis)
  var rotationPlane = Plane.fromNormalAndPoint(rotationAxis, rotationCenter)
  var orthobasis = new OrthoNormalBasis(rotationPlane)
  var transformation = Matrix4x4.translation(rotationCenter.negated())
  transformation = transformation.multiply(orthobasis.getProjectionMatrix())
  transformation = transformation.multiply(Matrix4x4.rotationZ(degrees))
  transformation = transformation.multiply(orthobasis.getInverseProjectionMatrix())
  transformation = transformation.multiply(Matrix4x4.translation(rotationCenter))
  return transformation
}

// Create an affine matrix for translation:
Matrix4x4.translation = function (v) {
    // parse as Vector3D, so we can pass an array or a Vector3D
  var vec = new Vector3D(v)
  var els = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, vec.x, vec.y, vec.z, 1]
  return new Matrix4x4(els)
}

// Create an affine matrix for mirroring into an arbitrary plane:
Matrix4x4.mirroring = function (plane) {
  var nx = plane.normal.x
  var ny = plane.normal.y
  var nz = plane.normal.z
  var w = plane.w
  var els = [
        (1.0 - 2.0 * nx * nx), (-2.0 * ny * nx), (-2.0 * nz * nx), 0,
        (-2.0 * nx * ny), (1.0 - 2.0 * ny * ny), (-2.0 * nz * ny), 0,
        (-2.0 * nx * nz), (-2.0 * ny * nz), (1.0 - 2.0 * nz * nz), 0,
        (2.0 * nx * w), (2.0 * ny * w), (2.0 * nz * w), 1
  ]
  return new Matrix4x4(els)
}

// Create an affine matrix for scaling:
Matrix4x4.scaling = function (v) {
    // parse as Vector3D, so we can pass an array or a Vector3D
  var vec = new Vector3D(v)
  var els = [
    vec.x, 0, 0, 0, 0, vec.y, 0, 0, 0, 0, vec.z, 0, 0, 0, 0, 1
  ]
  return new Matrix4x4(els)
}

module.exports = Matrix4x4
