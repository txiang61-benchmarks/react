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
import gnu.trove.iterator.TObjectIntIterator;
import gnu.trove.list.TIntList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;

import com.flowpowered.react.body.CollisionBody;
import com.flowpowered.react.collision.CollisionDetection;
import com.flowpowered.react.collision.shape.AABB;
import com.flowpowered.react.math.Vector3;

/**
 * This class implements the Sweep-And-Prune (SAP) broad-phase collision detection algorithm. This class implements an array-based version of the algorithm from Pierre Terdiman that is described here:
 * www.codercorner.com/SAP.pdf.
 */
public class SweepAndPruneAlgorithm extends BroadPhaseAlgorithm {
    private static final @Dimensionless int INVALID_INDEX = ((@Dimensionless int) (Integer.MAX_VALUE));
    private static final @Dimensionless int NB_SENTINELS = ((@Dimensionless int) (2));
    private @Dimensionless BoxAABB @Dimensionless [] mBoxes = null;
    private final @Dimensionless EndPoint @Dimensionless [] @Dimensionless [] mEndPoints = new @Dimensionless EndPoint @Dimensionless [] @Dimensionless [] {null, null, null};
    private @Dimensionless int mNbBoxes = ((@Dimensionless int) (0));
    private @Dimensionless int mNbMaxBoxes = ((@Dimensionless int) (0));
    private final @Dimensionless TIntList mFreeBoxIndices = new @Dimensionless TIntArrayList();
    private final @Dimensionless TObjectIntMap<@Dimensionless CollisionBody> mMapBodyToBoxIndex = new @Dimensionless TObjectIntHashMap<>();

    /**
     * Constructs a new sweep and prune algorithm from the collision detection it's associated to.
     *
     * @param collisionDetection The collision detection
     */
    public SweepAndPruneAlgorithm(CollisionDetection collisionDetection) {
        super(collisionDetection);
    }

    /**
     * Gets the number of objects managed by this algorithm.
     *
     * @return The number of objects
     */
    public @Dimensionless int getNbObjects(@Dimensionless SweepAndPruneAlgorithm this) {
        return mNbBoxes;
    }

    @Override
    public void addObject(@Dimensionless SweepAndPruneAlgorithm this, @Dimensionless CollisionBody body, @Dimensionless AABB aabb) {
        if (body == null) {
            throw new @Dimensionless IllegalArgumentException("Attempting to add a null collision body");
        }
        if (aabb == null) {
            throw new @Dimensionless IllegalArgumentException("Attempting to add a null AABB");
        }
        final @Dimensionless Vector3 extend = Vector3.subtract(aabb.getMax(), aabb.getMin());
        if (extend.getX() < ((@Dimensionless int) (0)) || extend.getY() < ((@Dimensionless int) (0)) || extend.getZ() < ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalStateException("AABB for body: " + body + " is invalid! AABB is " + aabb);
        }
        final @Dimensionless int boxIndex;
        if (mFreeBoxIndices.size() != ((@Dimensionless int) (0))) {
            boxIndex = mFreeBoxIndices.removeAt(mFreeBoxIndices.size() - ((@Dimensionless int) (1)));
        } else {
            if (mNbBoxes == mNbMaxBoxes) {
                resizeArrays();
            }
            boxIndex = mNbBoxes;
        }
        final @Dimensionless int indexLimitEndPoint = ((@Dimensionless int) (2)) * mNbBoxes + NB_SENTINELS - ((@Dimensionless int) (1));
        for (@Dimensionless int axis = ((@Dimensionless int) (0)); axis < ((@Dimensionless int) (3)); axis++) {
            final @Dimensionless EndPoint maxLimitEndPoint = mEndPoints[axis][indexLimitEndPoint];
            if (mEndPoints[axis][((@Dimensionless int) (0))].getBoxID() != INVALID_INDEX || !mEndPoints[axis][((@Dimensionless int) (0))].isMin()) {
                throw new @Dimensionless IllegalStateException("The box ID for the first end point of the axis must" +
                        " be equal to INVALID_INDEX and the end point must be a minimum");
            }
            if (maxLimitEndPoint.getBoxID() != INVALID_INDEX || maxLimitEndPoint.isMin()) {
                throw new @Dimensionless IllegalStateException("The box ID for the limit end point of the axis must" +
                        " be equal to INVALID_INDEX and the end point must be a maximum");
            }
            if (mEndPoints[axis][indexLimitEndPoint + ((@Dimensionless int) (2))] == null) {
                mEndPoints[axis][indexLimitEndPoint + ((@Dimensionless int) (2))] = new @Dimensionless EndPoint();
            }
            final @Dimensionless EndPoint newMaxLimitEndPoint = mEndPoints[axis][indexLimitEndPoint + ((@Dimensionless int) (2))];
            newMaxLimitEndPoint.setValues(maxLimitEndPoint.getBoxID(), maxLimitEndPoint.isMin(),
                    maxLimitEndPoint.getValue());
        }
        if (mBoxes[boxIndex] == null) {
            mBoxes[boxIndex] = new @Dimensionless BoxAABB();
        }
        final @Dimensionless BoxAABB box = mBoxes[boxIndex];
        box.setBody(body);
        final @Dimensionless long maxEndPointValue = encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))) - ((@Dimensionless int) (1));
        final @Dimensionless long minEndPointValue = encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))) - ((@Dimensionless int) (2));
        for (@Dimensionless int axis = ((@Dimensionless int) (0)); axis < ((@Dimensionless int) (3)); axis++) {
            box.getMin()[axis] = indexLimitEndPoint;
            box.getMax()[axis] = indexLimitEndPoint + ((@Dimensionless int) (1));
            final @Dimensionless EndPoint minimumEndPoint = mEndPoints[axis][box.getMin()[axis]];
            minimumEndPoint.setValues(boxIndex, true, minEndPointValue);
            if (mEndPoints[axis][box.getMax()[axis]] == null) {
                mEndPoints[axis][box.getMax()[axis]] = new @Dimensionless EndPoint();
            }
            final @Dimensionless EndPoint maximumEndPoint = mEndPoints[axis][box.getMax()[axis]];
            maximumEndPoint.setValues(boxIndex, false, maxEndPointValue);
        }
        mMapBodyToBoxIndex.put(body, boxIndex);
        mNbBoxes++;
        updateObject(body, aabb);
    }

    @Override
    public void removeObject(@Dimensionless SweepAndPruneAlgorithm this, @Dimensionless CollisionBody body) {
        final @Dimensionless long maxEndPointValue = encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))) - ((@Dimensionless int) (1));
        final @Dimensionless long minEndPointValue = encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE))) - ((@Dimensionless int) (2));
        final @Dimensionless AABBInt aabbInt = new @Dimensionless AABBInt(minEndPointValue, maxEndPointValue);
        updateObjectIntegerAABB(body, aabbInt);
        final @Dimensionless int boxIndex = mMapBodyToBoxIndex.get(body);
        final @Dimensionless int indexLimitEndPoint = ((@Dimensionless int) (2)) * mNbBoxes + NB_SENTINELS - ((@Dimensionless int) (1));
        for (@Dimensionless int axis = ((@Dimensionless int) (0)); axis < ((@Dimensionless int) (3)); axis++) {
            final @Dimensionless EndPoint maxLimitEndPoint = mEndPoints[axis][indexLimitEndPoint];
            if (mEndPoints[axis][((@Dimensionless int) (0))].getBoxID() != INVALID_INDEX || !mEndPoints[axis][((@Dimensionless int) (0))].isMin()) {
                throw new @Dimensionless IllegalStateException("The box ID for the first end point of the axis must" +
                        " be equal to INVALID_INDEX and the end point must be a minimum");
            }
            if (maxLimitEndPoint.getBoxID() != INVALID_INDEX || maxLimitEndPoint.isMin()) {
                throw new @Dimensionless IllegalStateException("The box ID for the limit end point of the axis must" +
                        " be equal to INVALID_INDEX and the end point must be a maximum");
            }
            final @Dimensionless EndPoint newMaxLimitEndPoint = mEndPoints[axis][indexLimitEndPoint - ((@Dimensionless int) (2))];
            newMaxLimitEndPoint.setValues(maxLimitEndPoint.getBoxID(), maxLimitEndPoint.isMin(), maxLimitEndPoint.getValue());
        }
        mFreeBoxIndices.add(boxIndex);
        mMapBodyToBoxIndex.remove(body);
        mNbBoxes--;
        final @Dimensionless int nextPowerOf2 = PairManager.computeNextPowerOfTwo((mNbBoxes - ((@Dimensionless int) (1))) / ((@Dimensionless int) (100)));
        if (nextPowerOf2 * ((@Dimensionless int) (100)) < mNbMaxBoxes) {
            shrinkArrays();
        }
    }

    @Override
    public void updateObject(@Dimensionless SweepAndPruneAlgorithm this, @Dimensionless CollisionBody body, @Dimensionless AABB aabb) {
        final @Dimensionless AABBInt aabbInt = new @Dimensionless AABBInt(aabb);
        updateObjectIntegerAABB(body, aabbInt);
    }

    public void updateObjectIntegerAABB(@Dimensionless SweepAndPruneAlgorithm this, @Dimensionless CollisionBody body, @Dimensionless AABBInt aabbInt) {
        final @Dimensionless int boxIndex = mMapBodyToBoxIndex.get(body);
        final @Dimensionless BoxAABB box = mBoxes[boxIndex];
        for (@Dimensionless int axis = ((@Dimensionless int) (0)); axis < ((@Dimensionless int) (3)); axis++) {
            final @Dimensionless int otherAxis1 = (((@Dimensionless int) (1)) << axis) & ((@Dimensionless int) (3));
            final @Dimensionless int otherAxis2 = (((@Dimensionless int) (1)) << otherAxis1) & ((@Dimensionless int) (3));
            final @Dimensionless EndPoint @Dimensionless [] startEndPointsCurrentAxis = mEndPoints[axis];
            // -------- Update the minimum end-point ------------//
            @Dimensionless
            EndPoint currentMinEndPoint = startEndPointsCurrentAxis[box.getMin()[axis]];
            @Dimensionless
            int currentMinEndPointIndex = box.getMin()[axis];
            if (!currentMinEndPoint.isMin()) {
                throw new @Dimensionless IllegalStateException("currentMinEndPoint must be a minimum");
            }
            @Dimensionless
            long limit = aabbInt.getMin()[axis];
            if (limit < currentMinEndPoint.getValue()) {
                final @Dimensionless EndPoint savedEndPoint = currentMinEndPoint;
                @Dimensionless
                int indexEndPoint = currentMinEndPointIndex;
                final @Dimensionless int savedEndPointIndex = indexEndPoint;
                currentMinEndPoint.setValue(limit);
                while ((currentMinEndPoint = startEndPointsCurrentAxis[--currentMinEndPointIndex]).getValue() > limit) {
                    final @Dimensionless BoxAABB id1 = mBoxes[currentMinEndPoint.getBoxID()];
                    final @Dimensionless boolean isMin = currentMinEndPoint.isMin();
                    if (!isMin) {
                        if (!box.equals(id1) && (box.getBody().isMotionEnabled() || id1.getBody().isMotionEnabled())) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2) &&
                                    testIntersect1DSortedAABBs(id1, aabbInt, startEndPointsCurrentAxis, axis)) {
                                mPairManager.addPair(body, id1.getBody());
                            }
                        }
                        id1.getMax()[axis] = indexEndPoint--;
                    } else {
                        id1.getMin()[axis] = indexEndPoint--;
                    }
                    startEndPointsCurrentAxis[currentMinEndPointIndex + ((@Dimensionless int) (1))] = currentMinEndPoint;
                }
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin()) {
                        mBoxes[savedEndPoint.getBoxID()].getMin()[axis] = indexEndPoint;
                    } else {
                        mBoxes[savedEndPoint.getBoxID()].getMax()[axis] = indexEndPoint;
                    }
                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            } else if (limit > currentMinEndPoint.getValue()) {
                final @Dimensionless EndPoint savedEndPoint = currentMinEndPoint;
                @Dimensionless
                int indexEndPoint = currentMinEndPointIndex;
                final @Dimensionless int savedEndPointIndex = indexEndPoint;
                currentMinEndPoint.setValue(limit);
                while ((currentMinEndPoint = startEndPointsCurrentAxis[++currentMinEndPointIndex]).getValue() < limit) {
                    final @Dimensionless BoxAABB id1 = mBoxes[currentMinEndPoint.getBoxID()];
                    final @Dimensionless boolean isMin = currentMinEndPoint.isMin();
                    if (!isMin) {
                        if (!box.equals(id1) && (box.getBody().isMotionEnabled() || id1.getBody().isMotionEnabled())) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)) {
                                mPairManager.removePair(body.getID(), id1.getBody().getID());
                            }
                        }
                        id1.getMax()[axis] = indexEndPoint++;
                    } else {
                        id1.getMin()[axis] = indexEndPoint++;
                    }
                    startEndPointsCurrentAxis[currentMinEndPointIndex - ((@Dimensionless int) (1))] = currentMinEndPoint;
                }
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin()) {
                        mBoxes[savedEndPoint.getBoxID()].getMin()[axis] = indexEndPoint;
                    } else {
                        mBoxes[savedEndPoint.getBoxID()].getMax()[axis] = indexEndPoint;
                    }
                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            }
            // ------- Update the maximum end-point ------------ //
            @Dimensionless
            EndPoint currentMaxEndPoint = startEndPointsCurrentAxis[box.getMax()[axis]];
            @Dimensionless
            int currentMaxEndPointIndex = box.getMax()[axis];
            if (currentMaxEndPoint.isMin()) {
                throw new @Dimensionless IllegalStateException("currentMinEndPoint must not be a minimum");
            }
            limit = aabbInt.getMax()[axis];
            if (limit > currentMaxEndPoint.getValue()) {
                final @Dimensionless EndPoint savedEndPoint = currentMaxEndPoint;
                @Dimensionless
                int indexEndPoint = currentMaxEndPointIndex;
                final @Dimensionless int savedEndPointIndex = indexEndPoint;
                currentMaxEndPoint.setValue(limit);
                while ((currentMaxEndPoint = startEndPointsCurrentAxis[++currentMaxEndPointIndex]).getValue() < limit) {
                    final @Dimensionless BoxAABB id1 = mBoxes[currentMaxEndPoint.getBoxID()];
                    final @Dimensionless boolean isMin = currentMaxEndPoint.isMin();
                    if (isMin) {
                        if (!box.equals(id1) && (box.getBody().isMotionEnabled() || id1.getBody().isMotionEnabled())) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2) &&
                                    testIntersect1DSortedAABBs(id1, aabbInt, startEndPointsCurrentAxis, axis)) {
                                mPairManager.addPair(body, id1.getBody());
                            }
                        }
                        id1.getMin()[axis] = indexEndPoint++;
                    } else {
                        id1.getMax()[axis] = indexEndPoint++;
                    }
                    startEndPointsCurrentAxis[currentMaxEndPointIndex - ((@Dimensionless int) (1))] = currentMaxEndPoint;
                }
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin()) {
                        mBoxes[savedEndPoint.getBoxID()].getMin()[axis] = indexEndPoint;
                    } else {
                        mBoxes[savedEndPoint.getBoxID()].getMax()[axis] = indexEndPoint;
                    }
                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            } else if (limit < currentMaxEndPoint.getValue()) {
                final @Dimensionless EndPoint savedEndPoint = currentMaxEndPoint;
                @Dimensionless
                int indexEndPoint = currentMaxEndPointIndex;
                final @Dimensionless int savedEndPointIndex = indexEndPoint;
                currentMaxEndPoint.setValue(limit);
                while ((currentMaxEndPoint = startEndPointsCurrentAxis[--currentMaxEndPointIndex]).getValue() > limit) {
                    final @Dimensionless BoxAABB id1 = mBoxes[currentMaxEndPoint.getBoxID()];
                    final @Dimensionless boolean isMin = currentMaxEndPoint.isMin();
                    if (isMin) {
                        if (!box.equals(id1) && (box.getBody().isMotionEnabled() || id1.getBody().isMotionEnabled())) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)) {
                                mPairManager.removePair(body.getID(), id1.getBody().getID());
                            }
                        }
                        id1.getMin()[axis] = indexEndPoint--;
                    } else {
                        id1.getMax()[axis] = indexEndPoint--;
                    }
                    startEndPointsCurrentAxis[currentMaxEndPointIndex + ((@Dimensionless int) (1))] = currentMaxEndPoint;
                }
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin()) {
                        mBoxes[savedEndPoint.getBoxID()].getMin()[axis] = indexEndPoint;
                    } else {
                        mBoxes[savedEndPoint.getBoxID()].getMax()[axis] = indexEndPoint;
                    }
                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            }
        }
    }

    // Re-sizes the boxes and end-points arrays when they are full.
    private void resizeArrays(@Dimensionless SweepAndPruneAlgorithm this) {
        final @Dimensionless int newNbMaxBoxes = mNbMaxBoxes != ((@Dimensionless int) (0)) ? ((@Dimensionless int) (2)) * mNbMaxBoxes : ((@Dimensionless int) (100));
        final @Dimensionless int nbEndPoints = mNbBoxes * ((@Dimensionless int) (2)) + NB_SENTINELS;
        final @Dimensionless int newNbEndPoints = newNbMaxBoxes * ((@Dimensionless int) (2)) + NB_SENTINELS;
        final @Dimensionless BoxAABB @Dimensionless [] newBoxesArray = new @Dimensionless BoxAABB @Dimensionless [newNbMaxBoxes];
        final @Dimensionless EndPoint @Dimensionless [] newEndPointsXArray = new @Dimensionless EndPoint @Dimensionless [newNbEndPoints];
        final @Dimensionless EndPoint @Dimensionless [] newEndPointsYArray = new @Dimensionless EndPoint @Dimensionless [newNbEndPoints];
        final @Dimensionless EndPoint @Dimensionless [] newEndPointsZArray = new @Dimensionless EndPoint @Dimensionless [newNbEndPoints];
        if (mNbBoxes > ((@Dimensionless int) (0))) {
            System.arraycopy(mBoxes, ((@Dimensionless int) (0)), newBoxesArray, ((@Dimensionless int) (0)), mNbBoxes);
            System.arraycopy(mEndPoints[((@Dimensionless int) (0))], ((@Dimensionless int) (0)), newEndPointsXArray, ((@Dimensionless int) (0)), nbEndPoints);
            System.arraycopy(mEndPoints[((@Dimensionless int) (1))], ((@Dimensionless int) (0)), newEndPointsYArray, ((@Dimensionless int) (0)), nbEndPoints);
            System.arraycopy(mEndPoints[((@Dimensionless int) (2))], ((@Dimensionless int) (0)), newEndPointsZArray, ((@Dimensionless int) (0)), nbEndPoints);
        } else {
            final @Dimensionless long min = encodeFloatIntoInteger(- ((@Dimensionless float) (Float.MAX_VALUE)));
            final @Dimensionless long max = encodeFloatIntoInteger(((@Dimensionless float) (Float.MAX_VALUE)));
            newEndPointsXArray[((@Dimensionless int) (0))] = new @Dimensionless EndPoint();
            newEndPointsXArray[((@Dimensionless int) (0))].setValues(INVALID_INDEX, true, min);
            newEndPointsXArray[((@Dimensionless int) (1))] = new @Dimensionless EndPoint();
            newEndPointsXArray[((@Dimensionless int) (1))].setValues(INVALID_INDEX, false, max);
            newEndPointsYArray[((@Dimensionless int) (0))] = new @Dimensionless EndPoint();
            newEndPointsYArray[((@Dimensionless int) (0))].setValues(INVALID_INDEX, true, min);
            newEndPointsYArray[((@Dimensionless int) (1))] = new @Dimensionless EndPoint();
            newEndPointsYArray[((@Dimensionless int) (1))].setValues(INVALID_INDEX, false, max);
            newEndPointsZArray[((@Dimensionless int) (0))] = new @Dimensionless EndPoint();
            newEndPointsZArray[((@Dimensionless int) (0))].setValues(INVALID_INDEX, true, min);
            newEndPointsZArray[((@Dimensionless int) (1))] = new @Dimensionless EndPoint();
            newEndPointsZArray[((@Dimensionless int) (1))].setValues(INVALID_INDEX, false, max);
        }
        mBoxes = newBoxesArray;
        mEndPoints[((@Dimensionless int) (0))] = newEndPointsXArray;
        mEndPoints[((@Dimensionless int) (1))] = newEndPointsYArray;
        mEndPoints[((@Dimensionless int) (2))] = newEndPointsZArray;
        mNbMaxBoxes = newNbMaxBoxes;
    }

    // Shrinks the boxes and end-points arrays when too much memory is allocated.
    private void shrinkArrays(@Dimensionless SweepAndPruneAlgorithm this) {
        final @Dimensionless int nextPowerOf2 = PairManager.computeNextPowerOfTwo((mNbBoxes - ((@Dimensionless int) (1))) / ((@Dimensionless int) (100)));
        final @Dimensionless int newNbMaxBoxes = mNbBoxes > ((@Dimensionless int) (100)) ? nextPowerOf2 * ((@Dimensionless int) (100)) : ((@Dimensionless int) (100));
        final @Dimensionless int nbEndPoints = mNbBoxes * ((@Dimensionless int) (2)) + NB_SENTINELS;
        final @Dimensionless int newNbEndPoints = newNbMaxBoxes * ((@Dimensionless int) (2)) + NB_SENTINELS;
        if (newNbMaxBoxes >= mNbMaxBoxes) {
            throw new @Dimensionless IllegalStateException("The new maximum number of boxes can't be greater or equal to the old one");
        }
        mFreeBoxIndices.sort();
        final @Dimensionless TObjectIntMap<@Dimensionless CollisionBody> newMapBodyToBoxIndex = new @Dimensionless TObjectIntHashMap<>();
        final @Dimensionless TObjectIntIterator<@Dimensionless CollisionBody> it = mMapBodyToBoxIndex.iterator();
        while (it.hasNext()) {
            it.advance();
            final @Dimensionless CollisionBody body = it.key();
            final @Dimensionless int boxIndex = it.value();
            if (boxIndex >= mNbBoxes) {
                if (mFreeBoxIndices.isEmpty()) {
                    throw new @Dimensionless IllegalStateException("The list of free box indices can't be empty");
                }
                final @Dimensionless int newBoxIndex = mFreeBoxIndices.removeAt(((@Dimensionless int) (0)));
                if (newBoxIndex >= mNbBoxes) {
                    throw new @Dimensionless IllegalStateException("The new box index can't be greater or equal to number of boxes");
                }
                final @Dimensionless BoxAABB oldBox = mBoxes[boxIndex];
                final @Dimensionless BoxAABB newBox = mBoxes[newBoxIndex];
                if (oldBox.getBody().getID() != body.getID()) {
                    throw new @Dimensionless IllegalStateException("The old box body ID can't be equal to body ID");
                }
                newBox.setBody(oldBox.getBody());
                for (@Dimensionless int axis = ((@Dimensionless int) (0)); axis < ((@Dimensionless int) (3)); axis++) {
                    newBox.setMin(axis, oldBox.getMin()[axis]);
                    newBox.setMax(axis, oldBox.getMax()[axis]);
                    final @Dimensionless EndPoint minimumEndPoint = mEndPoints[axis][newBox.getMin()[axis]];
                    final @Dimensionless EndPoint maximumEndPoint = mEndPoints[axis][newBox.getMax()[axis]];
                    if (minimumEndPoint.getBoxID() != boxIndex) {
                        throw new @Dimensionless IllegalStateException("The minimum end point box ID can't be equal to box index");
                    }
                    if (maximumEndPoint.getBoxID() != boxIndex) {
                        throw new @Dimensionless IllegalStateException("The maximum end point box ID can't be equal to box index");
                    }
                    minimumEndPoint.setBoxID(newBoxIndex);
                    maximumEndPoint.setBoxID(newBoxIndex);
                }
                newMapBodyToBoxIndex.put(body, newBoxIndex);
            } else {
                newMapBodyToBoxIndex.put(body, boxIndex);
            }
        }
        if (newMapBodyToBoxIndex.size() != mMapBodyToBoxIndex.size()) {
            throw new @Dimensionless IllegalStateException("The size of the new map from body to box index must be the same as the old one");
        }
        mMapBodyToBoxIndex.clear();
        mMapBodyToBoxIndex.putAll(newMapBodyToBoxIndex);
        final @Dimensionless BoxAABB @Dimensionless [] newBoxesArray = new @Dimensionless BoxAABB @Dimensionless [newNbMaxBoxes];
        final @Dimensionless EndPoint @Dimensionless [] newEndPointsXArray = new @Dimensionless EndPoint @Dimensionless [newNbEndPoints];
        final @Dimensionless EndPoint @Dimensionless [] newEndPointsYArray = new @Dimensionless EndPoint @Dimensionless [newNbEndPoints];
        final @Dimensionless EndPoint @Dimensionless [] newEndPointsZArray = new @Dimensionless EndPoint @Dimensionless [newNbEndPoints];
        System.arraycopy(mBoxes, ((@Dimensionless int) (0)), newBoxesArray, ((@Dimensionless int) (0)), mNbBoxes);
        System.arraycopy(mEndPoints[((@Dimensionless int) (0))], ((@Dimensionless int) (0)), newEndPointsXArray, ((@Dimensionless int) (0)), nbEndPoints);
        System.arraycopy(mEndPoints[((@Dimensionless int) (1))], ((@Dimensionless int) (0)), newEndPointsYArray, ((@Dimensionless int) (0)), nbEndPoints);
        System.arraycopy(mEndPoints[((@Dimensionless int) (2))], ((@Dimensionless int) (0)), newEndPointsZArray, ((@Dimensionless int) (0)), nbEndPoints);
        mBoxes = newBoxesArray;
        mEndPoints[((@Dimensionless int) (0))] = newEndPointsXArray;
        mEndPoints[((@Dimensionless int) (1))] = newEndPointsYArray;
        mEndPoints[((@Dimensionless int) (2))] = newEndPointsZArray;
        mNbMaxBoxes = newNbMaxBoxes;
    }

    // Encodes a floating value into a integer value in order to work with integer
    // comparisons in the Sweep-And-Prune algorithm, for performance.
    // The main issue when encoding a floating number into an integer is keeping
    // the sorting order. This is a problem for negative float numbers.
    // This article describes how to solve this issue: http://www.stereopsis.com/radix.html
    private static @Dimensionless long encodeFloatIntoInteger(@Dimensionless float number) {
        @Dimensionless
        long intNumber = (@Dimensionless long) Float.floatToIntBits(number) & ((@Dimensionless long) (0xFFFFFFFFl));
        if ((intNumber & ((@Dimensionless long) (0x80000000l))) == ((@Dimensionless long) (0x80000000l))) {
            intNumber = ~intNumber & ((@Dimensionless long) (0xFFFFFFFFl));
        } else {
            intNumber |= ((@Dimensionless long) (0x80000000l));
        }
        return intNumber;
    }

    // Checks for the intersection between two boxes that are sorted on the given axis in
    // one dimension. Only one test is necessary here. We know that the minimum of box1 cannot be
    // larger that the maximum of box2 on the axis.
    private static @Dimensionless boolean testIntersect1DSortedAABBs(@Dimensionless BoxAABB box1, @Dimensionless AABBInt box2, @Dimensionless EndPoint @Dimensionless [] endPointsArray, @Dimensionless int axis) {
        return !(endPointsArray[box1.getMax()[axis]].getValue() < box2.getMin()[axis]);
    }

    // Checks for intersection between two boxes in two dimensions. This method is used when
    // we know the that two boxes already overlap on one axis and when we want to test if they
    // also overlap on the two others axes.
    private static @Dimensionless boolean testIntersect2D(@Dimensionless BoxAABB box1, @Dimensionless BoxAABB box2, @Dimensionless int axis1, @Dimensionless int axis2) {
        return !(box2.getMax()[axis1] < box1.getMin()[axis1] || box1.getMax()[axis1] < box2.getMin()[axis1] ||
                box2.getMax()[axis2] < box1.getMin()[axis2] || box1.getMax()[axis2] < box2.getMin()[axis2]);
    }

    // Represents an end-point of an AABB on one of the three x,y or z axis.
    @Dimensionless
    private static class EndPoint {
        private @Dimensionless int boxID;
        private @Dimensionless boolean isMin;
        private @Dimensionless long value;

        private @Dimensionless int getBoxID(SweepAndPruneAlgorithm.@Dimensionless EndPoint this) {
            return boxID;
        }

        private void setBoxID(SweepAndPruneAlgorithm.@Dimensionless EndPoint this, @Dimensionless int boxID) {
            this.boxID = boxID;
        }

        private @Dimensionless boolean isMin(SweepAndPruneAlgorithm.@Dimensionless EndPoint this) {
            return isMin;
        }

        private @Dimensionless long getValue(SweepAndPruneAlgorithm.@Dimensionless EndPoint this) {
            return value;
        }

        private void setValue(SweepAndPruneAlgorithm.@Dimensionless EndPoint this, @Dimensionless long value) {
            this.value = value;
        }

        private void setValues(SweepAndPruneAlgorithm.@Dimensionless EndPoint this, @Dimensionless int boxID, @Dimensionless boolean isMin, @Dimensionless long value) {
            this.boxID = boxID;
            this.isMin = isMin;
            this.value = value;
        }

        @Override
        public @Dimensionless String toString(SweepAndPruneAlgorithm.@Dimensionless EndPoint this) {
            return "(" + value + "|" + isMin + "@" + boxID + ")";
        }
    public EndPoint(SweepAndPruneAlgorithm.@Dimensionless EndPoint this) { super(); }
    }

    // Represents an AABB in the Sweep-And-Prune algorithm.
    @Dimensionless
    private static class BoxAABB {
        private final @Dimensionless int @Dimensionless [] min = new @Dimensionless int @Dimensionless [((@Dimensionless int) (3))];
        private final @Dimensionless int @Dimensionless [] max = new @Dimensionless int @Dimensionless [((@Dimensionless int) (3))];
        private @Dimensionless CollisionBody body;

        private @Dimensionless int @Dimensionless [] getMin(SweepAndPruneAlgorithm.@Dimensionless BoxAABB this) {
            return min;
        }

        private void setMin(SweepAndPruneAlgorithm.@Dimensionless BoxAABB this, @Dimensionless int i, @Dimensionless int v) {
            min[i] = v;
        }

        private @Dimensionless int @Dimensionless [] getMax(SweepAndPruneAlgorithm.@Dimensionless BoxAABB this) {
            return max;
        }

        private void setMax(SweepAndPruneAlgorithm.@Dimensionless BoxAABB this, @Dimensionless int i, @Dimensionless int v) {
            max[i] = v;
        }

        private @Dimensionless CollisionBody getBody(SweepAndPruneAlgorithm.@Dimensionless BoxAABB this) {
            return body;
        }

        private void setBody(SweepAndPruneAlgorithm.@Dimensionless BoxAABB this, @Dimensionless CollisionBody body) {
            this.body = body;
        }

        @Override
        public @Dimensionless boolean equals(SweepAndPruneAlgorithm.@Dimensionless BoxAABB this, @Dimensionless Object o) {
            if (this == o) {
                return true;
            }
            if (!(o instanceof BoxAABB)) {
                return false;
            }
            final @Dimensionless BoxAABB boxAABB = (@Dimensionless BoxAABB) o;
            return !(body != null ? !body.equals(boxAABB.getBody()) : boxAABB.getBody() != null);
        }

        @Override
        public @Dimensionless int hashCode(SweepAndPruneAlgorithm.@Dimensionless BoxAABB this) {
            return body != null ? body.hashCode() : ((@Dimensionless int) (0));
        }

        @Override
        public @Dimensionless String toString(SweepAndPruneAlgorithm.@Dimensionless BoxAABB this) {
            return "(" + body.getID() + "@" + body.getTransform().getPosition() + ")";
        }
    public BoxAABB(SweepAndPruneAlgorithm.@Dimensionless BoxAABB this) { super(); }
    }

    // Represents an AABB with integer coordinates.
    @Dimensionless
    private static class AABBInt {
        private final @Dimensionless long @Dimensionless [] min = new @Dimensionless long @Dimensionless [((@Dimensionless int) (3))];
        private final @Dimensionless long @Dimensionless [] max = new @Dimensionless long @Dimensionless [((@Dimensionless int) (3))];

        private @Dimensionless long @Dimensionless [] getMin(SweepAndPruneAlgorithm.@Dimensionless AABBInt this) {
            return min;
        }

        private @Dimensionless long @Dimensionless [] getMax(SweepAndPruneAlgorithm.@Dimensionless AABBInt this) {
            return max;
        }

        private AABBInt(SweepAndPruneAlgorithm.@Dimensionless AABBInt this, @Dimensionless AABB aabb) {
            min[((@Dimensionless int) (0))] = encodeFloatIntoInteger(aabb.getMin().getX());
            min[((@Dimensionless int) (1))] = encodeFloatIntoInteger(aabb.getMin().getY());
            min[((@Dimensionless int) (2))] = encodeFloatIntoInteger(aabb.getMin().getZ());
            max[((@Dimensionless int) (0))] = encodeFloatIntoInteger(aabb.getMax().getX());
            max[((@Dimensionless int) (1))] = encodeFloatIntoInteger(aabb.getMax().getY());
            max[((@Dimensionless int) (2))] = encodeFloatIntoInteger(aabb.getMax().getZ());
        }

        private AABBInt(SweepAndPruneAlgorithm.@Dimensionless AABBInt this, @Dimensionless long minValue, @Dimensionless long maxValue) {
            min[((@Dimensionless int) (0))] = minValue;
            min[((@Dimensionless int) (1))] = minValue;
            min[((@Dimensionless int) (2))] = minValue;
            max[((@Dimensionless int) (0))] = maxValue;
            max[((@Dimensionless int) (1))] = maxValue;
            max[((@Dimensionless int) (2))] = maxValue;
        }
    }
}
