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
package com.flowpowered.react.collision.narrowphase.GJK;

import units.qual.Dimensionless;
import com.flowpowered.react.math.Vector3;

/**
 * Represents a simplex which is a set of 3D points. This class is used in the GJK algorithm. This implementation is based on the implementation discussed in the book "Collision Detection in 3D
 * Environments". This class implements the Johnson's algorithm for computing the point of a simplex that is closest to the origin and  the smallest simplex needed to represent that closest point.
 */
public class Simplex {
    private final @Dimensionless Vector3 @Dimensionless [] mPoints = new @Dimensionless Vector3 @Dimensionless [] {
            new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3()
    };
    private final @Dimensionless float @Dimensionless [] mPointsLengthSquare = new @Dimensionless float @Dimensionless [((@Dimensionless int) (4))];
    private @Dimensionless float mMaxLengthSquare;
    private final @Dimensionless Vector3 @Dimensionless [] mSuppPointsA = new @Dimensionless Vector3 @Dimensionless [] {
            new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3()
    };
    private final @Dimensionless Vector3 @Dimensionless [] mSuppPointsB = new @Dimensionless Vector3 @Dimensionless [] {
            new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3()
    };
    private final @Dimensionless Vector3 @Dimensionless [] @Dimensionless [] mDiffLength = new @Dimensionless Vector3 @Dimensionless [] @Dimensionless [] {
            new @Dimensionless Vector3 @Dimensionless [] {new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3()},
            new @Dimensionless Vector3 @Dimensionless [] {new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3()},
            new @Dimensionless Vector3 @Dimensionless [] {new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3()},
            new @Dimensionless Vector3 @Dimensionless [] {new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3(), new @Dimensionless Vector3()}
    };
    private final @Dimensionless float @Dimensionless [] @Dimensionless [] mDet = new @Dimensionless float @Dimensionless [((@Dimensionless int) (16))] @Dimensionless [((@Dimensionless int) (4))];
    private final @Dimensionless float @Dimensionless [] @Dimensionless [] mNormSquare = new @Dimensionless float @Dimensionless [((@Dimensionless int) (4))] @Dimensionless [((@Dimensionless int) (4))];
    /// 4 bits that identify the current points of the simplex
    /// For instance, 0101 means that points[1] and points[3] are in the simplex
    private @Dimensionless int mBitsCurrentSimplex = ((@Dimensionless int) (0x0));
    /// Number between 1 and 4 that identify the last found support point
    private @Dimensionless int mLastFound;
    /// Position of the last found support point (lastFoundBit = 0x1 << lastFound)
    private @Dimensionless int mLastFoundBit;
    /// allBits = bitsCurrentSimplex | lastFoundBit;
    private @Dimensionless int mAllBits = ((@Dimensionless int) (0x0));

    /**
     * Returns true if the simplex contains 4 points, false if not.
     *
     * @return Whether or not the simplex contains 4 points
     */
    public @Dimensionless boolean isFull(@Dimensionless Simplex this) {
        return mBitsCurrentSimplex == ((@Dimensionless int) (0xf));
    }

    /**
     * Returns true if the simple is empty, false if not
     *
     * @return Whether or not the simplex is empty
     */
    public @Dimensionless boolean isEmpty(@Dimensionless Simplex this) {
        return mBitsCurrentSimplex == ((@Dimensionless int) (0x0));
    }

    /**
     * Gets the maximum squared length of a point.
     *
     * @return The maximum squared length of a point
     */
    public float getMaxLengthSquareOfAPoint(@Dimensionless Simplex this) {
        return mMaxLengthSquare;
    }

    /**
     * Adds a new support point of (A-B) in the simplex.
     *
     * @param point The support point of object (A-B) => point = suppPointA - suppPointB
     * @param suppPointA The support point of object A in a direction -v
     * @param suppPointB The support point of object B in a direction v
     */
    public void addPoint(@Dimensionless Simplex this, @Dimensionless Vector3 point, @Dimensionless Vector3 suppPointA, @Dimensionless Vector3 suppPointB) {
        if (isFull()) {
            throw new @Dimensionless IllegalStateException("simplex is full");
        }
        mLastFound = ((@Dimensionless int) (0));
        mLastFoundBit = ((@Dimensionless int) (0x1));
        while (overlap(mBitsCurrentSimplex, mLastFoundBit)) {
            mLastFound++;
            mLastFoundBit <<= ((@Dimensionless int) (1));
        }
        if (mLastFound < ((@Dimensionless int) (0)) && mLastFound >= ((@Dimensionless int) (4))) {
            throw new @Dimensionless IllegalStateException("lastFount must be greater or equal to zero and smaller than four");
        }
        mPoints[mLastFound].set(point);
        mPointsLengthSquare[mLastFound] = point.dot(point);
        mAllBits = mBitsCurrentSimplex | mLastFoundBit;
        updateCache();
        computeDeterminants();
        mSuppPointsA[mLastFound].set(suppPointA);
        mSuppPointsB[mLastFound].set(suppPointB);
    }

    /**
     * Returns true if the point is in the simplex, false if not.
     *
     * @param point The point to verify for presence
     * @return Whether or not the point was found in the simplex
     */
    public @Dimensionless boolean isPointInSimplex(@Dimensionless Simplex this, @Dimensionless Vector3 point) {
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            if (overlap(mAllBits, bit) && point.equals(mPoints[i])) {
                return true;
            }
        }
        return false;
    }

    // Updates the cached values used during the GJK algorithm.
    private void updateCache(@Dimensionless Simplex this) {
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            if (overlap(mBitsCurrentSimplex, bit)) {
                mDiffLength[i][mLastFound] = Vector3.subtract(mPoints[i], mPoints[mLastFound]);
                mDiffLength[mLastFound][i] = Vector3.negate(mDiffLength[i][mLastFound]);
                mNormSquare[i][mLastFound] = mNormSquare[mLastFound][i] = mDiffLength[i][mLastFound].dot(mDiffLength[i][mLastFound]);
            }
        }
    }

    /**
     * Gets the points of the simplex. These will be stored in the arrays. The returned integer is the number of vertices.
     *
     * @param suppPointsA The support points of object A in a direction -v
     * @param suppPointsB The support points of object B in a direction v
     * @param points The support points of object (A-B) => point = suppPointA - suppPointB
     * @return The number of vertices
     */
    public int getSimplex(@Dimensionless Simplex this, Vector3[] suppPointsA, Vector3[] suppPointsB, Vector3[] points) {
        @Dimensionless
        int nbVertices = ((@Dimensionless int) (0));
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            if (overlap(mBitsCurrentSimplex, bit)) {
                suppPointsA[nbVertices] = new @Dimensionless Vector3(mSuppPointsA[nbVertices]);
                suppPointsB[nbVertices] = new @Dimensionless Vector3(mSuppPointsB[nbVertices]);
                points[nbVertices] = new @Dimensionless Vector3(mPoints[nbVertices]);
                nbVertices++;
            }
        }
        return nbVertices;
    }

    // Computes the cached determinant values.
    private void computeDeterminants(@Dimensionless Simplex this) {
        mDet[mLastFoundBit][mLastFound] = ((@Dimensionless int) (1));
        if (!isEmpty()) {
            for (int i = ((@Dimensionless int) (0)), bitI = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bitI <<= ((@Dimensionless int) (1))) {
                if (overlap(mBitsCurrentSimplex, bitI)) {
                    final @Dimensionless int bit2 = bitI | mLastFoundBit;
                    mDet[bit2][i] = mDiffLength[mLastFound][i].dot(mPoints[mLastFound]);
                    mDet[bit2][mLastFound] = mDiffLength[i][mLastFound].dot(mPoints[i]);
                    for (int j = ((@Dimensionless int) (0)), bitJ = ((@Dimensionless int) (0x1)); j < i; j++, bitJ <<= ((@Dimensionless int) (1))) {
                        if (overlap(mBitsCurrentSimplex, bitJ)) {
                            @Dimensionless
                            int k;
                            @Dimensionless
                            int bit3 = bitJ | bit2;
                            k = mNormSquare[i][j] < mNormSquare[mLastFound][j] ? i : mLastFound;
                            mDet[bit3][j] = mDet[bit2][i] * mDiffLength[k][j].dot(mPoints[i])
                                    + mDet[bit2][mLastFound] * mDiffLength[k][j].dot(mPoints[mLastFound]);
                            k = mNormSquare[j][i] < mNormSquare[mLastFound][i] ? j : mLastFound;
                            mDet[bit3][i] = mDet[bitJ | mLastFoundBit][j] * mDiffLength[k][i].dot(mPoints[j])
                                    + mDet[bitJ | mLastFoundBit][mLastFound] * mDiffLength[k][i].dot(mPoints[mLastFound]);
                            k = mNormSquare[i][mLastFound] < mNormSquare[j][mLastFound] ? i : j;
                            mDet[bit3][mLastFound] = mDet[bitJ | bitI][j] * mDiffLength[k][mLastFound].dot(mPoints[j])
                                    + mDet[bitJ | bitI][i] * mDiffLength[k][mLastFound].dot(mPoints[i]);
                        }
                    }
                }
            }
            if (mAllBits == ((@Dimensionless int) (0xf))) {
                @Dimensionless
                int k;
                k = mNormSquare[((@Dimensionless int) (1))][((@Dimensionless int) (0))] < mNormSquare[((@Dimensionless int) (2))][((@Dimensionless int) (0))] ?
                        (mNormSquare[((@Dimensionless int) (1))][((@Dimensionless int) (0))] < mNormSquare[((@Dimensionless int) (3))][((@Dimensionless int) (0))] ? ((@Dimensionless int) (1)) : ((@Dimensionless int) (3))) :
                        (mNormSquare[((@Dimensionless int) (2))][((@Dimensionless int) (0))] < mNormSquare[((@Dimensionless int) (3))][((@Dimensionless int) (0))] ? ((@Dimensionless int) (2)) : ((@Dimensionless int) (3)));
                mDet[((@Dimensionless int) (0xf))][((@Dimensionless int) (0))] = mDet[((@Dimensionless int) (0xe))][((@Dimensionless int) (1))] * mDiffLength[k][((@Dimensionless int) (0))].dot(mPoints[((@Dimensionless int) (1))])
                        + mDet[((@Dimensionless int) (0xe))][((@Dimensionless int) (2))] * mDiffLength[k][((@Dimensionless int) (0))].dot(mPoints[((@Dimensionless int) (2))])
                        + mDet[((@Dimensionless int) (0xe))][((@Dimensionless int) (3))] * mDiffLength[k][((@Dimensionless int) (0))].dot(mPoints[((@Dimensionless int) (3))]);
                k = mNormSquare[((@Dimensionless int) (0))][((@Dimensionless int) (1))] < mNormSquare[((@Dimensionless int) (2))][((@Dimensionless int) (1))] ?
                        (mNormSquare[((@Dimensionless int) (0))][((@Dimensionless int) (1))] < mNormSquare[((@Dimensionless int) (3))][((@Dimensionless int) (1))] ? ((@Dimensionless int) (0)) : ((@Dimensionless int) (3))) :
                        (mNormSquare[((@Dimensionless int) (2))][((@Dimensionless int) (1))] < mNormSquare[((@Dimensionless int) (3))][((@Dimensionless int) (1))] ? ((@Dimensionless int) (2)) : ((@Dimensionless int) (3)));
                mDet[((@Dimensionless int) (0xf))][((@Dimensionless int) (1))] = mDet[((@Dimensionless int) (0xd))][((@Dimensionless int) (0))] * mDiffLength[k][((@Dimensionless int) (1))].dot(mPoints[((@Dimensionless int) (0))])
                        + mDet[((@Dimensionless int) (0xd))][((@Dimensionless int) (2))] * mDiffLength[k][((@Dimensionless int) (1))].dot(mPoints[((@Dimensionless int) (2))])
                        + mDet[((@Dimensionless int) (0xd))][((@Dimensionless int) (3))] * mDiffLength[k][((@Dimensionless int) (1))].dot(mPoints[((@Dimensionless int) (3))]);
                k = mNormSquare[((@Dimensionless int) (0))][((@Dimensionless int) (2))] < mNormSquare[((@Dimensionless int) (1))][((@Dimensionless int) (2))] ?
                        (mNormSquare[((@Dimensionless int) (0))][((@Dimensionless int) (2))] < mNormSquare[((@Dimensionless int) (3))][((@Dimensionless int) (2))] ? ((@Dimensionless int) (0)) : ((@Dimensionless int) (3))) :
                        (mNormSquare[((@Dimensionless int) (1))][((@Dimensionless int) (2))] < mNormSquare[((@Dimensionless int) (3))][((@Dimensionless int) (2))] ? ((@Dimensionless int) (1)) : ((@Dimensionless int) (3)));
                mDet[((@Dimensionless int) (0xf))][((@Dimensionless int) (2))] = mDet[((@Dimensionless int) (0xb))][((@Dimensionless int) (0))] * mDiffLength[k][((@Dimensionless int) (2))].dot(mPoints[((@Dimensionless int) (0))])
                        + mDet[((@Dimensionless int) (0xb))][((@Dimensionless int) (1))] * mDiffLength[k][((@Dimensionless int) (2))].dot(mPoints[((@Dimensionless int) (1))])
                        + mDet[((@Dimensionless int) (0xb))][((@Dimensionless int) (3))] * mDiffLength[k][((@Dimensionless int) (2))].dot(mPoints[((@Dimensionless int) (3))]);
                k = mNormSquare[((@Dimensionless int) (0))][((@Dimensionless int) (3))] < mNormSquare[((@Dimensionless int) (1))][((@Dimensionless int) (3))] ?
                        (mNormSquare[((@Dimensionless int) (0))][((@Dimensionless int) (3))] < mNormSquare[((@Dimensionless int) (2))][((@Dimensionless int) (3))] ? ((@Dimensionless int) (0)) : ((@Dimensionless int) (2))) :
                        (mNormSquare[((@Dimensionless int) (1))][((@Dimensionless int) (3))] < mNormSquare[((@Dimensionless int) (2))][((@Dimensionless int) (3))] ? ((@Dimensionless int) (1)) : ((@Dimensionless int) (2)));
                mDet[((@Dimensionless int) (0xf))][((@Dimensionless int) (3))] = mDet[((@Dimensionless int) (0x7))][((@Dimensionless int) (0))] * mDiffLength[k][((@Dimensionless int) (3))].dot(mPoints[((@Dimensionless int) (0))])
                        + mDet[((@Dimensionless int) (0x7))][((@Dimensionless int) (1))] * mDiffLength[k][((@Dimensionless int) (3))].dot(mPoints[((@Dimensionless int) (1))])
                        + mDet[((@Dimensionless int) (0x7))][((@Dimensionless int) (2))] * mDiffLength[k][((@Dimensionless int) (3))].dot(mPoints[((@Dimensionless int) (2))]);
            }
        }
    }

    // Returns true if the subset is a proper subset.
    // A proper subset X is a subset where for all point "y_i" in X, we have detX_i value bigger than zero.
    private @Dimensionless boolean isProperSubset(@Dimensionless Simplex this, @Dimensionless int subset) {
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            if (overlap(subset, bit) && mDet[subset][i] <= ((@Dimensionless int) (0))) {
                return false;
            }
        }
        return true;
    }

    /**
     * Returns true if the set is affinely dependent. A set if affinely dependent if a point of the set is an affine combination of other points in the set.
     *
     * @return Whether or not the set is affinely dependent
     */
    public @Dimensionless boolean isAffinelyDependent(@Dimensionless Simplex this) {
        @Dimensionless
        float sum = ((@Dimensionless int) (0));
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            if (overlap(mAllBits, bit)) {
                sum += mDet[mAllBits][i];
            }
        }
        return sum <= ((@Dimensionless int) (0));
    }

    // Returns true if the subset is a valid one for the closest point computation.
    // A subset X is valid if:
    //    1. delta(X)_i > 0 for each "i" in I_x and
    //    2. delta(X U {y_j})_j <= 0 for each "j" not in I_x_
    private @Dimensionless boolean isValidSubset(@Dimensionless Simplex this, @Dimensionless int subset) {
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            if (overlap(mAllBits, bit)) {
                if (overlap(subset, bit)) {
                    if (mDet[subset][i] <= ((@Dimensionless int) (0))) {
                        return false;
                    }
                } else if (mDet[subset | bit][i] > ((@Dimensionless int) (0))) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Computes the closest points "pA" and "pB" of objects A and B.
     *
     * @param pA sum(lambda_i * a_i), where "a_i" is the support point of object A, with lambda_i = deltaX_i / deltaX
     * @param pB sum(lambda_i * b_i), where "b_i" is the support point of object B, with lambda_i = deltaX_i / deltaX
     */
    public void computeClosestPointsOfAAndB(@Dimensionless Simplex this, @Dimensionless Vector3 pA, @Dimensionless Vector3 pB) {
        @Dimensionless
        float deltaX = ((@Dimensionless int) (0));
        pA.setAllValues(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        pB.setAllValues(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            if (overlap(mBitsCurrentSimplex, bit)) {
                deltaX += mDet[mBitsCurrentSimplex][i];
                pA.add(Vector3.multiply(mDet[mBitsCurrentSimplex][i], mSuppPointsA[i]));
                pB.add(Vector3.multiply(mDet[mBitsCurrentSimplex][i], mSuppPointsB[i]));
            }
        }
        if (deltaX <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalStateException("deltaX must be greater than zero");
        }
        final @Dimensionless float factor = ((@Dimensionless int) (1)) / deltaX;
        pA.multiply(factor);
        pB.multiply(factor);
    }

    /**
     * Compute the closest point "v" to the origin of the current simplex. This method executes the Johnson's algorithm for computing the point "v" of a simplex that is the closest to the origin. The
     * method returns true if a closest point has been found, false if not. The closest point is store in the passed vector, if found.
     *
     * @param v The vector in which to store the closest point
     * @return Whether or not the closest point has been found
     */
    public @Dimensionless boolean computeClosestPoint(@Dimensionless Simplex this, @Dimensionless Vector3 v) {
        for (@Dimensionless int subset = mBitsCurrentSimplex; subset != ((@Dimensionless int) (0x0)); subset--) {
            if (isSubset(subset, mBitsCurrentSimplex) && isValidSubset(subset | mLastFoundBit)) {
                mBitsCurrentSimplex = subset | mLastFoundBit;
                v.set(computeClosestPointForSubset(mBitsCurrentSimplex));
                return true;
            }
        }
        if (isValidSubset(mLastFoundBit)) {
            mBitsCurrentSimplex = mLastFoundBit;
            mMaxLengthSquare = mPointsLengthSquare[mLastFound];
            v.set(mPoints[mLastFound]);
            return true;
        }
        return false;
    }

    /**
     * Backups the closest point to the passed vector.
     *
     * @param v The vector in which to store the closest point
     */
    public void backupClosestPointInSimplex(@Dimensionless Simplex this, @Dimensionless Vector3 v) {
        @Dimensionless
        float minDistSquare = ((@Dimensionless float) (Float.MAX_VALUE));
        for (@Dimensionless int bit = mAllBits; bit != ((@Dimensionless int) (0x0)); bit--) {
            if (isSubset(bit, mAllBits) && isProperSubset(bit)) {
                final @Dimensionless Vector3 u = computeClosestPointForSubset(bit);
                final @Dimensionless float distSquare = u.dot(u);
                if (distSquare < minDistSquare) {
                    minDistSquare = distSquare;
                    mBitsCurrentSimplex = bit;
                    v.set(u);
                }
            }
        }
    }

    // Returns the closest point "v" in the convex hull of the points in the subset represented
    // by the bits "subset".
    private @Dimensionless Vector3 computeClosestPointForSubset(@Dimensionless Simplex this, @Dimensionless int subset) {
        final @Dimensionless Vector3 v = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        mMaxLengthSquare = ((@Dimensionless int) (0));
        @Dimensionless
        float deltaX = ((@Dimensionless int) (0));
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            if (overlap(subset, bit)) {
                deltaX += mDet[subset][i];
                if (mMaxLengthSquare < mPointsLengthSquare[i]) {
                    mMaxLengthSquare = mPointsLengthSquare[i];
                }
                v.add(Vector3.multiply(mDet[subset][i], mPoints[i]));
            }
        }
        if (deltaX <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalStateException("deltaX must be greater than zero");
        }
        return Vector3.multiply(((@Dimensionless int) (1)) / deltaX, v);
    }

    // Returns true if some bits of "a" overlap with bits of "b".
    private static @Dimensionless boolean overlap(@Dimensionless int a, @Dimensionless int b) {
        return (a & b) != ((@Dimensionless int) (0x0));
    }

    // Returns true if the bits of "b" is a subset of the bits of "a".
    private static @Dimensionless boolean isSubset(@Dimensionless int a, @Dimensionless int b) {
        return (a & b) == a;
    }
}
