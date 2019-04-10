/* eslint-disable object-shorthand,semi,no-else-return */
// //////////////////////////////
// ## class fuzzyFactory
// This class acts as a factory for objects. We can search for an object with approximately
// the desired properties (say a rectangle with width 2 and height 1)
// The lookupOrCreate() method looks for an existing object (for example it may find an existing rectangle
// with width 2.0001 and height 0.999. If no object is found, the user supplied callback is
// called, which should generate a new object. The new object is inserted into the database
// so it can be found by future lookupOrCreate() calls.
// Constructor:
//   numdimensions: the number of parameters for each object
//     for example for a 2D rectangle this would be 2
//   tolerance: The maximum difference for each parameter allowed to be considered a match
//
// ael adaptation: the original code stored an object using a set of hashes made with
// [floor, ceil] choices for each element.  for example, a point near 2, 1 might be
// stored under the four hashes "2000/999", "2000/1000", "2001/999", "2001/1000".  this
// makes the hashing more forgiving, but at potentially huge cost - for example, when
// dealing with the six property values of a vertex with position and normal.  the new
// property "multipleHash", set false by default, controls this behaviour.
const FuzzyFactory = function (numdimensions, tolerance, multipleHash) {
  this.lookuptable = {}
  this.multiplier = Math.round(1.0 / tolerance)  // ael - might as well use an integer
  this.multipleHash = !!multipleHash

  this.lookups = this.hits = 0 // diagnostic info, if wanted.  for example, in canonicalizeCSG you could put console.log((factory.vertexfactory.hits/factory.vertexfactory.lookups).toFixed(2))
}

FuzzyFactory.prototype = {
    // let obj = f.lookupOrCreate([el1, el2, el3], function(elements) {/* create the new object */});
    // Performs a fuzzy lookup of the object with the specified elements.
    // If found, returns the existing object
    // If not found, calls the supplied callback function which should create a new object with
    // the specified properties. This object is inserted in the lookup database.
  lookupOrCreate: function (els, creatorCallback) {
    this.lookups++

    let hash = ''
    let multiplier = this.multiplier
    els.forEach(function (el) {
      let valueQuantized = Math.round(el * multiplier)
      hash += valueQuantized + '/'
    })

    if (hash in this.lookuptable) {
      this.hits++;
      return this.lookuptable[hash]
    } else {
      let object = creatorCallback(els)
      if (this.multipleHash) {
        let hashparts = els.map(function (el) {
          let q0 = Math.floor(el * multiplier)
          let q1 = q0 + 1
          return ['' + q0 + '/', '' + q1 + '/']
        })
        let numelements = els.length
        let numhashes = 1 << numelements
        for (let hashmask = 0; hashmask < numhashes; ++hashmask) {
          let hashmaskShifted = hashmask
          hash = ''
          hashparts.forEach(function (hashpart) {
            hash += hashpart[hashmaskShifted & 1]
            hashmaskShifted >>= 1
          })
          this.lookuptable[hash] = object
        }
      } else this.lookuptable[hash] = object

      return object
    }
  }
}

module.exports = FuzzyFactory
