/*
 * This file is part of React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://danielchappuis.ch>
 * React is re-licensed with permission from ReactPhysics3D author.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package com.flowpowered.react.collision.broadphase;

import units.qual.Dimensionless;
import com.flowpowered.react.Utilities;
import com.flowpowered.react.Utilities.IntPair;
import com.flowpowered.react.body.CollisionBody;
import com.flowpowered.react.collision.CollisionDetection;

/**
 * This class is a data-structure contains the pairs of bodies that are overlapping during the broad-phase collision detection. This class implements the pair manager described by Pierre Terdiman in
 * www.codercorner.com/SAP.pdf.
 */
public class PairManager {
    private static final @Dimensionless int INVALID_INDEX = ((@Dimensionless int) (Integer.MAX_VALUE));
    private @Dimensionless int mNbElementsHashTable = ((@Dimensionless int) (0));
    private @Dimensionless int mHashMask = ((@Dimensionless int) (0));
    private @Dimensionless int mNbOverlappingPairs = ((@Dimensionless int) (0));
    private @Dimensionless int @Dimensionless [] mHashTable = null;
    private @Dimensionless int @Dimensionless [] mOffsetNextPair = null;
    private @Dimensionless BodyPair @Dimensionless [] mOverlappingPairs = null;
    private final @Dimensionless CollisionDetection mCollisionDetection;

    /**
     * Constructs a new pair manager from the collision detection it's associated to.
     *
     * @param collisionDetection The associated collision detection
     */
    public PairManager(CollisionDetection collisionDetection) {
        mCollisionDetection = collisionDetection;
    }

    /**
     * Gets the number of overlapping pairs.
     *
     * @return The number of overlapping pairs
     */
    public @Dimensionless long getNbOverlappingPairs(@Dimensionless PairManager this) {
        return mNbOverlappingPairs;
    }

    // Compute the hash value of two bodies
    private @Dimensionless int computeHashBodies(@Dimensionless PairManager this, @Dimensionless int id1, @Dimensionless int id2) {
        return computeHash32Bits(id1 | id2 << ((@Dimensionless int) (16)));
    }

    // Return true if pair1 and pair2 are the same
    private @Dimensionless boolean isDifferentPair(@Dimensionless PairManager this, @Dimensionless BodyPair pair1, @Dimensionless int pair2ID1, @Dimensionless int pair2ID2) {
        return pair2ID1 != pair1.getFirstBody().getID() || pair2ID2 != pair1.getSecondBody().getID();
    }

    // Sort the bodies according to their IDs (smallest ID first)
    private @Dimensionless BodyPair sortBodiesUsingID(@Dimensionless PairManager this, @Dimensionless CollisionBody body1, @Dimensionless CollisionBody body2) {
        if (body1.getID() > body2.getID()) {
            final @Dimensionless CollisionBody temp = body2;
            body2 = body1;
            body1 = temp;
        }
        return new @Dimensionless BodyPair(body1, body2);
    }

    // Sort the IDs (smallest ID first)
    private @Dimensionless IntPair sortIDs(@Dimensionless PairManager this, @Dimensionless int id1, @Dimensionless int id2) {
        if (id1 > id2) {
            final @Dimensionless int temp = id2;
            id2 = id1;
            id1 = temp;
        }
        return new @Dimensionless IntPair(id1, id2);
    }

    // This method returns an hash value for a 32 bits key.
    // using Thomas Wang's hash technique.
    // This hash function can be found at :
    // http://www.concentric.net/~ttwang/tech/inthash.htm
    private @Dimensionless int computeHash32Bits(@Dimensionless PairManager this, @Dimensionless int key) {
        key += ~(key << ((@Dimensionless int) (15)));
        key ^= key >> ((@Dimensionless int) (10));
        key += key << ((@Dimensionless int) (3));
        key ^= key >> ((@Dimensionless int) (6));
        key += ~(key << ((@Dimensionless int) (11)));
        key ^= key >> ((@Dimensionless int) (16));
        return key;
    }

    /**
     * Finds a pair for the two body IDs. Returns null if no pair with the bodies having the IDs can be found.
     *
     * @param id1 The first ID
     * @param id2 The second ID
     * @return The pair, or null if none can be found
     */
    public @Dimensionless BodyPair findPair(@Dimensionless PairManager this, @Dimensionless int id1, @Dimensionless int id2) {
        if (mHashTable == null) {
            return null;
        }
        sortIDs(id1, id2);
        final @Dimensionless int hashValue = computeHashBodies(id1, id2) & mHashMask;
        return lookForAPair(id1, id2, hashValue);
    }

    // Find a pair given two body IDs and an hash value.
    // This internal version is used to avoid computing multiple times in the
    // caller method
    private @Dimensionless BodyPair findPairWithHashValue(@Dimensionless PairManager this, @Dimensionless int id1, @Dimensionless int id2, @Dimensionless int hashValue) {
        if (mHashTable == null) {
            return null;
        }
        return lookForAPair(id1, id2, hashValue);
    }

    // Try to reduce the allocated memory by the pair manager
    private void shrinkMemory(@Dimensionless PairManager this) {
        final @Dimensionless int correctNbElementsHashTable = computeNextPowerOfTwo(mNbOverlappingPairs);
        if (mNbElementsHashTable == correctNbElementsHashTable) {
            return;
        }
        mNbElementsHashTable = correctNbElementsHashTable;
        mHashMask = mNbElementsHashTable - ((@Dimensionless int) (1));
        reallocatePairs();
    }

    // Compute the offset of a given pair in the array of overlapping pairs
    private @Dimensionless int computePairOffset(@Dimensionless PairManager this, @Dimensionless BodyPair pair) {
        final @Dimensionless int offset = Utilities.indexOf(mOverlappingPairs, pair);
        if (offset == ((@Dimensionless int) (-1))) {
            throw new @Dimensionless IllegalArgumentException("pair not in mOverlappingPairs[]");
        }
        return offset;
    }

    /**
     * Returns the array of overlapping pairs managed by this class, for iteration purposes. Note that the array returned contains trailing null elements.
     *
     * @return The array of overlapping pairs
     */
    public BodyPair[] getOverlappingPairs(@Dimensionless PairManager this) {
        return mOverlappingPairs;
    }

    /**
     * Return the last overlapping pair (used to iterate over the overlapping pairs) or returns null if there are no overlapping pairs. Note that the array returned by {@link #getOverlappingPairs()}
     * contains trailing null elements.
     *
     * @return The last overlapping pair
     */
    public BodyPair getLastOverlappingPair(@Dimensionless PairManager this) {
        if (mNbOverlappingPairs > ((@Dimensionless int) (0))) {
            return mOverlappingPairs[mNbOverlappingPairs - ((@Dimensionless int) (1))];
        } else {
            return null;
        }
    }

    /**
     * Adds a pair of bodies in the pair manager and returns the pair. If the pair to add does not already exist in the set of overlapping pairs, it will be created. If it already exists, the pair is
     * returned only.
     *
     * @param body1 The first body of the pair
     * @param body2 The second boy of the pair
     * @return The added pair
     */
    public BodyPair addPair(@Dimensionless PairManager this, CollisionBody body1, CollisionBody body2) {
        final @Dimensionless BodyPair newPair = sortBodiesUsingID(body1, body2);
        body1 = newPair.getFirstBody();
        body2 = newPair.getSecondBody();
        final @Dimensionless int id1 = body1.getID();
        final @Dimensionless int id2 = body2.getID();
        @Dimensionless
        int hashValue = computeHashBodies(id1, id2) & mHashMask;
        final @Dimensionless BodyPair pair = findPairWithHashValue(id1, id2, hashValue);
        if (pair != null) {
            return pair;
        }
        if (mNbOverlappingPairs >= mNbElementsHashTable) {
            mNbElementsHashTable = computeNextPowerOfTwo(mNbOverlappingPairs + ((@Dimensionless int) (1)));
            mHashMask = mNbElementsHashTable - ((@Dimensionless int) (1));
            reallocatePairs();
            hashValue = computeHashBodies(id1, id2) & mHashMask;
        }
        mOverlappingPairs[mNbOverlappingPairs] = newPair;
        mOffsetNextPair[mNbOverlappingPairs] = mHashTable[hashValue];
        mHashTable[hashValue] = mNbOverlappingPairs++;
        mCollisionDetection.broadPhaseNotifyAddedOverlappingPair(newPair);
        return newPair;
    }

    /**
     * Removes a pair of bodies from the pair manager. Returns true if the pair has been found and removed, false if not.
     *
     * @param id1 The ID of the first body in the pair
     * @param id2 The ID of the second body in the pair
     * @return Whether or not the pair has been removed
     */
    public boolean removePair(@Dimensionless PairManager this, int id1, int id2) {
        final @Dimensionless IntPair intPair = sortIDs(id1, id2);
        id1 = intPair.getFirst();
        id2 = intPair.getSecond();
        final @Dimensionless int hashValue = computeHashBodies(id1, id2) & mHashMask;
        final @Dimensionless BodyPair pair = findPairWithHashValue(id1, id2, hashValue);
        if (pair == null) {
            return false;
        }
        if (pair.getFirstBody().getID() != id1) {
            throw new @Dimensionless IllegalStateException("Incorrect pair was found");
        }
        if (pair.getSecondBody().getID() != id2) {
            throw new @Dimensionless IllegalStateException("Incorrect pair was found");
        }
        mCollisionDetection.broadPhaseNotifyRemovedOverlappingPair(pair);
        removePairWithHashValue(hashValue, computePairOffset(pair));
        shrinkMemory();
        return true;
    }

    // Internal method to remove a pair from the set of overlapping pair
    private void removePairWithHashValue(@Dimensionless PairManager this, @Dimensionless int hashValue, @Dimensionless int indexPair) {
        @Dimensionless
        int offset = mHashTable[hashValue];
        if (offset == INVALID_INDEX) {
            throw new @Dimensionless IllegalStateException("offset cannot be equal to INVALID_INDEX");
        }
        @Dimensionless
        int previousPair = INVALID_INDEX;
        while (offset != indexPair) {
            previousPair = offset;
            offset = mOffsetNextPair[offset];
        }
        if (previousPair == INVALID_INDEX) {
            mHashTable[hashValue] = mOffsetNextPair[indexPair];
        } else {
            if (mOffsetNextPair[previousPair] != indexPair) {
                throw new @Dimensionless IllegalStateException("offsetNextPair at index previousPair must be equal to indexPair");
            }
            mOffsetNextPair[previousPair] = mOffsetNextPair[indexPair];
        }
        final @Dimensionless int indexLastPair = mNbOverlappingPairs - ((@Dimensionless int) (1));
        if (indexPair == indexLastPair) {
            mNbOverlappingPairs--;
        } else {
            final @Dimensionless BodyPair lastPair = mOverlappingPairs[indexLastPair];
            final @Dimensionless int lastPairHashValue = computeHashBodies(lastPair.getFirstBody().getID(), lastPair.getSecondBody().getID()) & mHashMask;
            offset = mHashTable[lastPairHashValue];
            if (offset == INVALID_INDEX) {
                throw new @Dimensionless IllegalStateException("offset cannot be equal to INVALID_INDEX");
            }
            @Dimensionless
            int previous = INVALID_INDEX;
            while (offset != indexLastPair) {
                previous = offset;
                offset = mOffsetNextPair[offset];
            }
            if (previous != INVALID_INDEX) {
                if (mOffsetNextPair[previous] != indexLastPair) {
                    throw new @Dimensionless IllegalStateException("offsetNextPair at index previous must be equal to indexLastPair");
                }
                mOffsetNextPair[previous] = mOffsetNextPair[indexLastPair];
            } else {
                mHashTable[lastPairHashValue] = mOffsetNextPair[indexLastPair];
            }
            mOverlappingPairs[indexPair] = mOverlappingPairs[indexLastPair];
            mOffsetNextPair[indexPair] = mHashTable[lastPairHashValue];
            mHashTable[lastPairHashValue] = indexPair;

            mNbOverlappingPairs--;
        }
    }

    // Look for a pair in the set of overlapping pairs
    private @Dimensionless BodyPair lookForAPair(@Dimensionless PairManager this, @Dimensionless int id1, @Dimensionless int id2, @Dimensionless int hashValue) {
        @Dimensionless
        int offset = mHashTable[hashValue];
        while (offset != INVALID_INDEX && isDifferentPair(mOverlappingPairs[offset], id1, id2)) {
            offset = mOffsetNextPair[offset];
        }
        if (offset == INVALID_INDEX) {
            return null;
        }
        if (offset >= mNbOverlappingPairs) {
            throw new @Dimensionless IllegalStateException("offset cannot be greater or equal to nbOverlappingPairs");
        }
        return mOverlappingPairs[offset];
    }

    // Reallocate more pairs
    private void reallocatePairs() {
        mHashTable = new @Dimensionless int @Dimensionless [mNbElementsHashTable];
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbElementsHashTable; i++) {
            mHashTable[i] = INVALID_INDEX;
        }
        final @Dimensionless BodyPair @Dimensionless [] newOverlappingPairs = new @Dimensionless BodyPair @Dimensionless [mNbElementsHashTable];
        @Dimensionless
        int @Dimensionless [] newOffsetNextPair = new @Dimensionless int @Dimensionless [mNbElementsHashTable];
        if (mNbOverlappingPairs != ((@Dimensionless int) (0))) {
            System.arraycopy(mOverlappingPairs, ((@Dimensionless int) (0)), newOverlappingPairs, ((@Dimensionless int) (0)), mNbOverlappingPairs);
        }
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbOverlappingPairs; i++) {
            final @Dimensionless int newHashValue = computeHashBodies(mOverlappingPairs[i].getFirstBody().getID(),
                    mOverlappingPairs[i].getSecondBody().getID()) & mHashMask;
            newOffsetNextPair[i] = mHashTable[newHashValue];
            mHashTable[newHashValue] = i;
        }
        mOverlappingPairs = newOverlappingPairs;
        mOffsetNextPair = newOffsetNextPair;
    }

    /**
     * Returns the next power of two of a 32bit integer using a SWAR algorithm.
     *
     * @param number The number
     * @return The next power of two
     */
    public static @Dimensionless int computeNextPowerOfTwo(@Dimensionless int number) {
        number |= number >> ((@Dimensionless int) (1));
        number |= number >> ((@Dimensionless int) (2));
        number |= number >> ((@Dimensionless int) (4));
        number |= number >> ((@Dimensionless int) (8));
        number |= number >> ((@Dimensionless int) (16));
        return number + ((@Dimensionless int) (1));
    }

    /**
     * Represents a pair of bodies during the broad-phase collision detection.
     */
    public static class BodyPair {
        private @Dimensionless CollisionBody body1;
        private @Dimensionless CollisionBody body2;

        /**
         * Default constructor. Both bodies are null.
         */
        public BodyPair() {
        }

        /**
         * Constructs a new body pair from the first and second body.
         *
         * @param body1 The first body
         * @param body2 The second body
         */
        public BodyPair(PairManager.@Dimensionless BodyPair this, @Dimensionless CollisionBody body1, @Dimensionless CollisionBody body2) {
            this.body1 = body1;
            this.body2 = body2;
        }

        /**
         * Gets the first body in the pair.
         *
         * @return The first body
         */
        public @Dimensionless CollisionBody getFirstBody(PairManager.@Dimensionless BodyPair this) {
            return body1;
        }

        /**
         * Sets the first body to the desired value.
         *
         * @param body1 The body to set
         */
        public void setFirstBody(PairManager.@Dimensionless BodyPair this, @Dimensionless CollisionBody body1) {
            this.body1 = body1;
        }

        /**
         * Gets the second body in the pair.
         *
         * @return The second body
         */
        public @Dimensionless CollisionBody getSecondBody(PairManager.@Dimensionless BodyPair this) {
            return body2;
        }

        /**
         * Sets the second body to the desired value.
         *
         * @param body2 The body to set
         */
        public void setSecondBody(PairManager.@Dimensionless BodyPair this, @Dimensionless CollisionBody body2) {
            this.body2 = body2;
        }

        /**
         * Gets the indexes (IDs) of the bodies as a pair of integers.
         *
         * @return The pair of integer for the indexes (IDs)
         */
        public @Dimensionless IntPair getBodiesIndexPair(PairManager.@Dimensionless BodyPair this) {
            final @Dimensionless IntPair indexPair = body1.getID() < body2.getID() ?
                    new @Dimensionless IntPair(body1.getID(), body2.getID()) :
                    new @Dimensionless IntPair(body2.getID(), body1.getID());
            if (indexPair.getFirst() == indexPair.getSecond()) {
                throw new @Dimensionless IllegalStateException("The first body of the index pair cannot be equal to the second");
            }
            return indexPair;
        }
    }
}
