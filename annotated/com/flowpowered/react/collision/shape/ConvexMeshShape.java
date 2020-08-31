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
package com.flowpowered.react.collision.shape;

import units.qual.Dimensionless;
import java.util.ArrayList;
import java.util.List;

import gnu.trove.iterator.TIntIterator;
import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.set.TIntSet;
import gnu.trove.set.hash.TIntHashSet;

import com.flowpowered.react.ReactDefaults;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Vector3;

/**
 * This class represents a convex mesh shape. In order to create a convex mesh shape, you need to indicate the local-space position of the mesh vertices. You do this either by passing a vertices array
 * to the constructor or using the addVertex() method. Make sure that the set of vertices that you use to create the shape are indeed part of a convex mesh. The center of mass of the shape will be at
 * the origin of the local-space geometry that you use to create the mesh. The method used for collision detection with a convex mesh shape has an O(n) running time with "n" being the number of
 * vertices in the mesh. Therefore, you should try not to use too many vertices. However, it is possible to speed up the collision detection by using the edges information of your mesh. The running
 * time of the collision detection that uses the edges is almost O(1) constant time at the cost of additional memory used to store the vertices. You can indicate edges information with the addEdge()
 * method. Then, you must use the setIsEdgesInformationUsed(true) method in order to use the edges information for collision detection.
 */
@Dimensionless
public class ConvexMeshShape extends CollisionShape {
    private final @Dimensionless List<@Dimensionless Vector3> mVertices = new @Dimensionless ArrayList<>();
    private @Dimensionless int mNbVertices;
    private final @Dimensionless Vector3 mMinBounds;
    private final @Dimensionless Vector3 mMaxBounds;
    private @Dimensionless boolean mIsEdgesInformationUsed;
    private final @Dimensionless TIntObjectMap<@Dimensionless TIntSet> mEdgesAdjacencyList = new @Dimensionless TIntObjectHashMap<>();
    private @Dimensionless int mCachedSupportVertex;

    /**
     * Constructs a new convex mesh shape from the array of vertices, the number of vertices in the mesh and the stride in bytes (the amount of bytes per vertex)
     *
     * @param arrayVertices The array of vertices
     * @param nbVertices The number of vertices in the mesh
     * @param stride The vertex stride in bytes
     */
    public ConvexMeshShape(@Dimensionless float @Dimensionless [] arrayVertices, @Dimensionless int nbVertices, @Dimensionless int stride) {
        this(arrayVertices, nbVertices, stride, ReactDefaults.OBJECT_MARGIN);
    }

    /**
     * Constructs a new convex mesh shape from the array of vertices, the number of vertices in the mesh, the stride in bytes (the amount of bytes per vertex) and the collision margin.
     *
     * @param arrayVertices The array of vertices
     * @param nbVertices The number of vertices in the mesh
     * @param stride The vertex stride in bytes
     * @param margin The collision margin
     */
    public ConvexMeshShape(@Dimensionless float @Dimensionless [] arrayVertices, @Dimensionless int nbVertices, @Dimensionless int stride, @Dimensionless float margin) {
        super(CollisionShapeType.CONVEX_MESH, margin);
        mNbVertices = nbVertices;
        mMinBounds = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        mMaxBounds = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        mIsEdgesInformationUsed = false;
        mCachedSupportVertex = ((@Dimensionless int) (0));
        if (nbVertices <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Number of vertices must be greater than zero");
        }
        if (stride <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Stride must be greater than zero");
        }
        if (margin <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Margin must be greater than 0");
        }
        @Dimensionless
        int vertexPointer = ((@Dimensionless int) (0));
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbVertices; i++) {
            final @Dimensionless int newPoint = vertexPointer / ((@Dimensionless int) (4));
            mVertices.add(new @Dimensionless Vector3(arrayVertices[newPoint], arrayVertices[newPoint + ((@Dimensionless int) (1))], arrayVertices[newPoint + ((@Dimensionless int) (2))]));
            vertexPointer += stride;
        }
        recalculateBounds();
    }

    /**
     * Constructs a new convex mesh shape with no mesh.
     */
    public ConvexMeshShape() {
        this(ReactDefaults.OBJECT_MARGIN);
    }

    /**
     * Constructs a new convex mesh shape with no mesh from the collision margin.
     *
     * @param margin The collision margin
     */
    public ConvexMeshShape(@Dimensionless float margin) {
        super(CollisionShapeType.CONVEX_MESH, margin);
        mNbVertices = ((@Dimensionless int) (0));
        mMinBounds = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        mMaxBounds = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        mIsEdgesInformationUsed = false;
        mCachedSupportVertex = ((@Dimensionless int) (0));
        if (margin <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Margin must be greater than 0");
        }
    }

    /**
     * Copy constructor.
     *
     * @param shape The shape to copy
     */
    public ConvexMeshShape(@Dimensionless ConvexMeshShape shape) {
        super(shape);
        mVertices.addAll(shape.mVertices);
        mNbVertices = shape.mNbVertices;
        mMinBounds = new @Dimensionless Vector3(shape.mMinBounds);
        mMaxBounds = new @Dimensionless Vector3(shape.mMaxBounds);
        mIsEdgesInformationUsed = shape.mIsEdgesInformationUsed;
        mEdgesAdjacencyList.putAll(shape.mEdgesAdjacencyList);
        mCachedSupportVertex = shape.mCachedSupportVertex;
        if (mNbVertices != mVertices.size()) {
            throw new @Dimensionless IllegalArgumentException("The number of vertices must be equal to the size of the vertex list");
        }
    }

    // Recomputes the bounds of the mesh.
    private void recalculateBounds(@Dimensionless ConvexMeshShape this) {
        mMinBounds.setToZero();
        mMaxBounds.setToZero();
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbVertices; i++) {
            if (mVertices.get(i).getX() > mMaxBounds.getX()) {
                mMaxBounds.setX(mVertices.get(i).getX());
            }
            if (mVertices.get(i).getX() < mMinBounds.getX()) {
                mMinBounds.setX(mVertices.get(i).getX());
            }
            if (mVertices.get(i).getY() > mMaxBounds.getY()) {
                mMaxBounds.setY(mVertices.get(i).getY());
            }
            if (mVertices.get(i).getY() < mMinBounds.getY()) {
                mMinBounds.setY(mVertices.get(i).getY());
            }
            if (mVertices.get(i).getZ() > mMaxBounds.getZ()) {
                mMaxBounds.setZ(mVertices.get(i).getZ());
            }
            if (mVertices.get(i).getZ() < mMinBounds.getZ()) {
                mMinBounds.setZ(mVertices.get(i).getZ());
            }
        }
        mMaxBounds.add(new @Dimensionless Vector3(mMargin, mMargin, mMargin));
        mMinBounds.subtract(new @Dimensionless Vector3(mMargin, mMargin, mMargin));
    }

    /**
     * Adds a vertex into the convex mesh.
     *
     * @param vertex The vertex to add
     */
    public void addVertex(@Dimensionless ConvexMeshShape this, @Dimensionless Vector3 vertex) {
        mVertices.add(vertex);
        mNbVertices++;
        if (vertex.getX() > mMaxBounds.getX()) {
            mMaxBounds.setX(vertex.getX());
        }
        if (vertex.getX() < mMinBounds.getX()) {
            mMinBounds.setX(vertex.getX());
        }
        if (vertex.getY() > mMaxBounds.getY()) {
            mMaxBounds.setY(vertex.getY());
        }
        if (vertex.getY() < mMinBounds.getY()) {
            mMinBounds.setY(vertex.getY());
        }
        if (vertex.getZ() > mMaxBounds.getZ()) {
            mMaxBounds.setZ(vertex.getZ());
        }
        if (vertex.getZ() < mMinBounds.getZ()) {
            mMinBounds.setZ(vertex.getZ());
        }
    }

    /**
     * Adds an edge into the convex mesh by specifying the two vertex indices of the edge. Note that the vertex indices start at zero and need to correspond to the order of the vertices in the vertex
     * array in the constructor or the order of the calls of the addVertex() methods that were used to add vertices into the convex mesh.
     *
     * @param v1 The first vertex of the edge
     * @param v2 The second vertex of the edge
     */
    public void addEdge(@Dimensionless ConvexMeshShape this, @Dimensionless int v1, @Dimensionless int v2) {
        if (v1 < ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("v1 must be greater or equal to zero");
        }
        if (v2 < ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("v2 must be greater or equal to zero");
        }
        if (!mEdgesAdjacencyList.containsKey(v1)) {
            mEdgesAdjacencyList.put(v1, new @Dimensionless TIntHashSet());
        }
        if (!mEdgesAdjacencyList.containsKey(v2)) {
            mEdgesAdjacencyList.put(v2, new @Dimensionless TIntHashSet());
        }
        mEdgesAdjacencyList.get(v1).add(v2);
        mEdgesAdjacencyList.get(v2).add(v1);
    }

    /**
     * Returns true if the edges information is used to speed up the collision detection.
     *
     * @return Whether or not the edge information is used
     */
    public @Dimensionless boolean isEdgesInformationUsed(@Dimensionless ConvexMeshShape this) {
        return mIsEdgesInformationUsed;
    }

    /**
     * Sets the variable to know if the edges information is used to speed up the collision detection.
     *
     * @param isEdgesUsed Whether or not to use the edge information
     */
    public void setIsEdgesInformationUsed(@Dimensionless ConvexMeshShape this, @Dimensionless boolean isEdgesUsed) {
        mIsEdgesInformationUsed = isEdgesUsed;
    }

    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithMargin(@Dimensionless ConvexMeshShape this, @Dimensionless Vector3 direction) {
        final @Dimensionless Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction);
        final @Dimensionless Vector3 unitDirection = new @Dimensionless Vector3(direction);
        if (direction.lengthSquare() < ReactDefaults.MACHINE_EPSILON * ReactDefaults.MACHINE_EPSILON) {
            unitDirection.setAllValues(((@Dimensionless int) (1)), ((@Dimensionless int) (1)), ((@Dimensionless int) (1)));
        }
        unitDirection.normalize();
        return Vector3.add(supportPoint, Vector3.multiply(unitDirection, mMargin));
    }

    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithoutMargin(@Dimensionless ConvexMeshShape this, @Dimensionless Vector3 direction) {
        if (mNbVertices != mVertices.size()) {
            throw new @Dimensionless IllegalArgumentException("The number of vertices must be equal to the size of the vertex list");
        }
        if (mIsEdgesInformationUsed) {
            if (mEdgesAdjacencyList.size() != mNbVertices) {
                throw new @Dimensionless IllegalStateException("The number of adjacent edge lists must be equal to the number of vertices");
            }
            @Dimensionless
            int maxVertex = mCachedSupportVertex;
            @Dimensionless
            float maxDotProduct = direction.dot(mVertices.get(maxVertex));
            @Dimensionless
            boolean isOptimal;
            do {
                isOptimal = true;
                final @Dimensionless TIntSet edgeSet = mEdgesAdjacencyList.get(maxVertex);
                if (edgeSet.size() <= ((@Dimensionless int) (0))) {
                    throw new @Dimensionless IllegalStateException("The number of adjacent edges must be greater than zero");
                }
                final @Dimensionless TIntIterator it = edgeSet.iterator();
                while (it.hasNext()) {
                    final @Dimensionless int i = it.next();
                    final @Dimensionless float dotProduct = direction.dot(mVertices.get(i));
                    if (dotProduct > maxDotProduct) {
                        maxVertex = i;
                        maxDotProduct = dotProduct;
                        isOptimal = false;
                    }
                }
            } while (!isOptimal);
            mCachedSupportVertex = maxVertex;
            return mVertices.get(maxVertex);
        } else {
            @Dimensionless
            float maxDotProduct = - ((@Dimensionless float) (Float.MAX_VALUE));
            @Dimensionless
            int indexMaxDotProduct = ((@Dimensionless int) (0));
            for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mNbVertices; i++) {
                final @Dimensionless float dotProduct = direction.dot(mVertices.get(i));
                if (dotProduct > maxDotProduct) {
                    indexMaxDotProduct = i;
                    maxDotProduct = dotProduct;
                }
            }
            if (maxDotProduct < ((@Dimensionless int) (0))) {
                throw new @Dimensionless IllegalStateException("Max dot product is not greater or equal to zero");
            }
            return mVertices.get(indexMaxDotProduct);
        }
    }

    @Override
    public void getLocalBounds(@Dimensionless ConvexMeshShape this, @Dimensionless Vector3 min, @Dimensionless Vector3 max) {
        min.set(mMinBounds);
        max.set(mMaxBounds);
    }

    @Override
    public void computeLocalInertiaTensor(@Dimensionless ConvexMeshShape this, @Dimensionless Matrix3x3 tensor, @Dimensionless float mass) {
        final @Dimensionless float factor = (((@Dimensionless float) (1f)) / ((@Dimensionless int) (3))) * mass;
        final @Dimensionless Vector3 realExtent = Vector3.multiply(((@Dimensionless float) (0.5f)), Vector3.subtract(mMaxBounds, mMinBounds));
        if (realExtent.getX() <= ((@Dimensionless int) (0)) || realExtent.getY() <= ((@Dimensionless int) (0)) || realExtent.getZ() <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalStateException("Real extent components must all be greater than zero");
        }
        final @Dimensionless float xSquare = realExtent.getX() * realExtent.getX();
        final @Dimensionless float ySquare = realExtent.getY() * realExtent.getY();
        final @Dimensionless float zSquare = realExtent.getZ() * realExtent.getZ();
        tensor.setAllValues(
                factor * (ySquare + zSquare), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), factor * (xSquare + zSquare), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), factor * (xSquare + ySquare));
    }

    @Override
    public @Dimensionless CollisionShape clone(@Dimensionless ConvexMeshShape this) {
        return new @Dimensionless ConvexMeshShape(this);
    }

    @Override
    public @Dimensionless boolean isEqualTo(@Dimensionless ConvexMeshShape this, @Dimensionless CollisionShape otherCollisionShape) {
        final @Dimensionless ConvexMeshShape otherShape = (@Dimensionless ConvexMeshShape) otherCollisionShape;
        return mNbVertices == otherShape.mNbVertices && !mIsEdgesInformationUsed && mVertices.equals(otherShape.mVertices) && mEdgesAdjacencyList.equals(otherShape.mVertices);
    }
}
