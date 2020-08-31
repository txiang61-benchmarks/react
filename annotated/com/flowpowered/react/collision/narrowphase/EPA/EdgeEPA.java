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
package com.flowpowered.react.collision.narrowphase.EPA;

import units.qual.Dimensionless;
import com.flowpowered.react.math.Vector3;

/**
 * Represents an edge for the current polytope in the EPA algorithm.
 */
public class EdgeEPA {
    private @Dimensionless TriangleEPA mOwnerTriangle;
    private @Dimensionless int mIndex;

    /**
     * Default constructor. The owner triangle is null and the index is zero.
     */
    public EdgeEPA() {
        this(null, ((@Dimensionless int) (0)));
    }

    /**
     * Construct a new edge for the EPA from its owner triangle and index.
     *
     * @param ownerTriangle The owner triangle
     * @param index The index
     */
    public EdgeEPA(TriangleEPA ownerTriangle, int index) {
        if (index < ((@Dimensionless int) (0)) || index >= ((@Dimensionless int) (3))) {
            throw new @Dimensionless IllegalArgumentException("index must be greater or equal to zero and smaller than three");
        }
        mOwnerTriangle = ownerTriangle;
        mIndex = index;
    }

    /**
     * Copy constructor.
     *
     * @param edge The edge to copy
     */
    public EdgeEPA(@Dimensionless EdgeEPA edge) {
        mOwnerTriangle = edge.mOwnerTriangle;
        mIndex = edge.mIndex;
    }

    /**
     * Gets the owner triangle.
     *
     * @return The owner triangle
     */
    public TriangleEPA getOwnerTriangle(@Dimensionless EdgeEPA this) {
        return mOwnerTriangle;
    }

    /**
     * Gets the edge index.
     *
     * @return The edge index
     */
    public int getIndex(@Dimensionless EdgeEPA this) {
        return mIndex;
    }

    /**
     * Sets the values of this edge to the ones of the provided edge.
     *
     * @param edge The edge to copy the values from
     */
    public void set(@Dimensionless EdgeEPA this, @Dimensionless EdgeEPA edge) {
        mOwnerTriangle = edge.mOwnerTriangle;
        mIndex = edge.mIndex;
    }

    /**
     * Gets the index of the source vertex for the edge (vertex starting the edge).
     *
     * @return The index of the source vertex
     */
    public int getSourceVertexIndex(@Dimensionless EdgeEPA this) {
        return mOwnerTriangle.get(mIndex);
    }

    /**
     * Gets the index of the target vertex for the edge (vertex ending the edge).
     *
     * @return The index of the target vertex
     */
    public int getTargetVertexIndex(@Dimensionless EdgeEPA this) {
        return mOwnerTriangle.get(indexOfNextCounterClockwiseEdge(mIndex));
    }

    /**
     * Executes the recursive silhouette algorithm from this edge.
     *
     * @param vertices The vertices
     * @param indexNewVertex The index of the new vertex
     * @param triangleStore The triangle store
     * @return True if the owner triangle was obsolete or if a new triangle edge was half liked with this one
     */
    public boolean computeSilhouette(@Dimensionless EdgeEPA this, Vector3[] vertices, int indexNewVertex, TrianglesStore triangleStore) {
        if (!mOwnerTriangle.isObsolete()) {
            if (!mOwnerTriangle.isVisibleFromVertex(vertices, indexNewVertex)) {
                @Dimensionless
                TriangleEPA triangle = triangleStore.newTriangle(
                        vertices,
                        indexNewVertex,
                        getTargetVertexIndex(),
                        getSourceVertexIndex());
                if (triangle != null) {
                    TriangleEPA.halfLink(new @Dimensionless EdgeEPA(triangle, ((@Dimensionless int) (1))), this);
                    return true;
                }
                return false;
            } else {
                mOwnerTriangle.setObsolete(true);
                @Dimensionless
                int backup = triangleStore.getNbTriangles();
                if (!mOwnerTriangle.getAdjacentEdge(indexOfNextCounterClockwiseEdge(this.mIndex))
                        .computeSilhouette(vertices, indexNewVertex, triangleStore)) {
                    mOwnerTriangle.setObsolete(false);
                    @Dimensionless
                    TriangleEPA triangle = triangleStore.newTriangle(
                            vertices,
                            indexNewVertex,
                            getTargetVertexIndex(),
                            getSourceVertexIndex());
                    if (triangle != null) {
                        TriangleEPA.halfLink(new @Dimensionless EdgeEPA(triangle, ((@Dimensionless int) (1))), this);
                        return true;
                    }
                    return false;
                } else if (!mOwnerTriangle.getAdjacentEdge(indexOfPreviousCounterClockwiseEdge(this.mIndex))
                        .computeSilhouette(vertices, indexNewVertex, triangleStore)) {
                    mOwnerTriangle.setObsolete(false);
                    triangleStore.setNbTriangles(backup);
                    @Dimensionless
                    TriangleEPA triangle = triangleStore.newTriangle(
                            vertices,
                            indexNewVertex,
                            getTargetVertexIndex(),
                            getSourceVertexIndex());
                    if (triangle != null) {
                        TriangleEPA.halfLink(new @Dimensionless EdgeEPA(triangle, ((@Dimensionless int) (1))), this);
                        return true;
                    }
                    return false;
                }
            }
        }
        return true;
    }

    // Returns the index of the next counter-clockwise edge of the owner triangle.
    private static @Dimensionless int indexOfNextCounterClockwiseEdge(@Dimensionless int i) {
        return (i + ((@Dimensionless int) (1))) % ((@Dimensionless int) (3));
    }

    // Returns the index of the previous counter-clockwise edge of the owner triangle.
    private static @Dimensionless int indexOfPreviousCounterClockwiseEdge(@Dimensionless int i) {
        return (i + ((@Dimensionless int) (2))) % ((@Dimensionless int) (3));
    }
}
