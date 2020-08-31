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
package com.flowpowered.react.math;

import units.qual.Dimensionless;
import java.util.Arrays;

/**
 * Represents a 4x4 matrix.
 */
@Dimensionless
public class Matrix4x4 {
    /**
     * The first row of the matrix. Value of 0.
     */
    public static final @Dimensionless int FIRST_ROW = ((@Dimensionless int) (0));
    /**
     * The second row of the matrix. Value of 1.
     */
    public static final @Dimensionless int SECOND_ROW = ((@Dimensionless int) (1));
    /**
     * The third row of the matrix. Value of 2.
     */
    public static final @Dimensionless int THIRD_ROW = ((@Dimensionless int) (2));
    /**
     * The fourth row of the matrix. Value of 3.
     */
    public static final @Dimensionless int FOURTH_ROW = ((@Dimensionless int) (3));
    /**
     * The first column of the matrix. Value of 0.
     */
    public static final @Dimensionless int FIRST_COLUMN = ((@Dimensionless int) (0));
    /**
     * The second column of the matrix. Value of 1.
     */
    public static final @Dimensionless int SECOND_COLUMN = ((@Dimensionless int) (1));
    /**
     * The third column of the matrix. Value of 2.
     */
    public static final @Dimensionless int THIRD_COLUMN = ((@Dimensionless int) (2));
    /**
     * The fourth column of the matrix. Value of 3.
     */
    public static final @Dimensionless int FOURTH_COLUMN = ((@Dimensionless int) (3));
    private final @Dimensionless Vector4 @Dimensionless [] mRows = new @Dimensionless Vector4 @Dimensionless [] {
            new @Dimensionless Vector4(),
            new @Dimensionless Vector4(),
            new @Dimensionless Vector4(),
            new @Dimensionless Vector4()
    };

    /**
     * Default constructor. All values are 0.
     */
    public Matrix4x4() {
        this(((@Dimensionless int) (0)));
    }

    /**
     * Constructs a new 4x4 matrix with all values set to the provided one.
     *
     * @param value The value to use
     */
    public Matrix4x4(@Dimensionless float value) {
        this(
                value, value, value, value,
                value, value, value, value,
                value, value, value, value,
                value, value, value, value);
    }

    /**
     * Copy constructor.
     *
     * @param matrix The matrix to copy
     */
    public Matrix4x4(@Dimensionless Matrix4x4 matrix) {
        this(
                matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))), matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))),
                matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))), matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))),
                matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))), matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))),
                matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))), matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))));
    }

    /**
     * Construct a new matrix from all the values.
     *
     * @param a1 The value for 0,0
     * @param a2 The value for 0,1
     * @param a3 The value for 0,2
     * @param a4 The value for 0,3
     * @param b1 The value for 1,0
     * @param b2 The value for 1,1
     * @param b3 The value for 1,2
     * @param b4 The value for 1,3
     * @param c1 The value for 2,0
     * @param c2 The value for 2,1
     * @param c3 The value for 2,2
     * @param c4 The value for 2,3
     * @param d1 The value for 3,0
     * @param d2 The value for 3,1
     * @param d3 The value for 3,2
     * @param d4 The value for 3,3
     */
    public Matrix4x4(@Dimensionless float a1, @Dimensionless float a2, @Dimensionless float a3, @Dimensionless float a4,
                     @Dimensionless
                     float b1, @Dimensionless float b2, @Dimensionless float b3, @Dimensionless float b4,
                     @Dimensionless
                     float c1, @Dimensionless float c2, @Dimensionless float c3, @Dimensionless float c4,
                     @Dimensionless
                     float d1, @Dimensionless float d2, @Dimensionless float d3, @Dimensionless float d4) {
        setAllValues(a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4, d1, d2, d3, d4);
    }

    /**
     * Sets all the values to the provided ones.
     *
     * @param a1 The value for 0,0
     * @param a2 The value for 0,1
     * @param a3 The value for 0,2
     * @param a4 The value for 0,3
     * @param b1 The value for 1,0
     * @param b2 The value for 1,1
     * @param b3 The value for 1,2
     * @param b4 The value for 1,3
     * @param c1 The value for 2,0
     * @param c2 The value for 2,1
     * @param c3 The value for 2,2
     * @param c4 The value for 2,3
     * @param d1 The value for 3,0
     * @param d2 The value for 3,1
     * @param d3 The value for 3,2
     * @param d4 The value for 3,3
     */
    public final void setAllValues(@Dimensionless Matrix4x4 this, @Dimensionless float a1, @Dimensionless float a2, @Dimensionless float a3, @Dimensionless float a4,
                                   @Dimensionless
                                   float b1, @Dimensionless float b2, @Dimensionless float b3, @Dimensionless float b4,
                                   @Dimensionless
                                   float c1, @Dimensionless float c2, @Dimensionless float c3, @Dimensionless float c4,
                                   @Dimensionless
                                   float d1, @Dimensionless float d2, @Dimensionless float d3, @Dimensionless float d4) {
        mRows[((@Dimensionless int) (0))].setAllValues(a1, a2, a3, a4);
        mRows[((@Dimensionless int) (1))].setAllValues(b1, b2, b3, b4);
        mRows[((@Dimensionless int) (2))].setAllValues(c1, c2, c3, c4);
        mRows[((@Dimensionless int) (3))].setAllValues(d1, d2, d3, d4);
    }

    /**
     * Sets the values of this matrix to those of the provided matrix.
     *
     * @param matrix The matrix to copy the values from
     * @return This matrix for chained calls
     */
    public @Dimensionless Matrix4x4 set(@Dimensionless Matrix4x4 this, @Dimensionless Matrix4x4 matrix) {
        setAllValues(
                matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))), matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))),
                matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))), matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))),
                matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))), matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))),
                matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))), matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))));
        return this;
    }

    /**
     * Gets the desired row as a vector4.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}, {@link #THIRD_ROW}, {@link #FOURTH_ROW}
     * @return The vector4 for the row
     */
    public @Dimensionless Vector4 get(@Dimensionless Matrix4x4 this, @Dimensionless int row) {
        return mRows[row];
    }

    /**
     * Gets the desired value at the row and column.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}, {@link #THIRD_ROW}, {@link #FOURTH_ROW}
     * @param col The column number, either {@link #FIRST_COLUMN}, {@link #SECOND_COLUMN}, {@link #THIRD_COLUMN}, {@link #FOURTH_COLUMN}
     * @return The value at the row and column
     */
    public @Dimensionless float get(@Dimensionless Matrix4x4 this, @Dimensionless int row, @Dimensionless int col) {
        return mRows[row].get(col);
    }

    /**
     * Gets the desired column as a vector3.
     *
     * @param col The column number, either {@link #FIRST_COLUMN}, {@link #SECOND_COLUMN}, {@link #THIRD_COLUMN}, {@link #FOURTH_COLUMN}
     * @return The column as a vector3
     */
    public @Dimensionless Vector4 getColumn(@Dimensionless Matrix4x4 this, @Dimensionless int col) {
        return new @Dimensionless Vector4(mRows[((@Dimensionless int) (0))].get(col), mRows[((@Dimensionless int) (1))].get(col), mRows[((@Dimensionless int) (2))].get(col), mRows[((@Dimensionless int) (3))].get(col));
    }

    /**
     * Gets the desired row as a vector4.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}, {@link #THIRD_ROW}, {@link #FOURTH_ROW}
     * @return The vector4 for the row
     */
    public @Dimensionless Vector4 getRow(@Dimensionless Matrix4x4 this, @Dimensionless int row) {
        return mRows[row];
    }

    /**
     * Sets the value at the row and column to the desired value.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}, {@link #THIRD_ROW}, {@link #FOURTH_ROW}
     * @param col The column number, either {@link #FIRST_COLUMN}, {@link #SECOND_COLUMN}, {@link #THIRD_COLUMN}, {@link #FOURTH_COLUMN}
     * @param value The value to set at the row and column
     */
    public void set(@Dimensionless Matrix4x4 this, @Dimensionless int row, @Dimensionless int col, @Dimensionless float value) {
        mRows[row].set(col, value);
    }

    /**
     * Sets all the values at zero.
     */
    public void setToZero(@Dimensionless Matrix4x4 this) {
        mRows[((@Dimensionless int) (0))].setToZero();
        mRows[((@Dimensionless int) (1))].setToZero();
        mRows[((@Dimensionless int) (2))].setToZero();
        mRows[((@Dimensionless int) (3))].setToZero();
    }

    /**
     * Sets all the matrix values to identity.
     */
    public void setToIdentity(@Dimensionless Matrix4x4 this) {
        mRows[((@Dimensionless int) (0))].setAllValues(((@Dimensionless int) (1)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        mRows[((@Dimensionless int) (1))].setAllValues(((@Dimensionless int) (0)), ((@Dimensionless int) (1)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        mRows[((@Dimensionless int) (2))].setAllValues(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (1)), ((@Dimensionless int) (0)));
        mRows[((@Dimensionless int) (3))].setAllValues(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (1)));
    }

    /**
     * Transposes the matrix to a new matrix and returns it.
     *
     * @return The new matrix which is the transposed version of this one
     */
    public @Dimensionless Matrix4x4 getTranspose(@Dimensionless Matrix4x4 this) {
        return new @Dimensionless Matrix4x4(
                mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))),
                mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))),
                mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))),
                mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))));
    }

    /**
     * Calculates the determinant of this matrix and returns it.
     *
     * @return The determinant of this matrix
     */
    public @Dimensionless float getDeterminant(@Dimensionless Matrix4x4 this) {
        @Dimensionless
        float det = mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) * (
                (mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3)))
                        + mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3)))
                        + mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))))
                        - mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3)))
                        - mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3)))
                        - mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))));
        det -= mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) * (
                (mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3)))
                        + mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3)))
                        + mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))))
                        - mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3)))
                        - mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3)))
                        - mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))));
        det += mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))) * (
                (mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3)))
                        + mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3)))
                        + mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))))
                        - mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3)))
                        - mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3)))
                        - mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))));
        det -= mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))) * (
                (mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3)))
                        + mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3)))
                        + mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))))
                        - mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3)))
                        - mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3)))
                        - mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))));
        return det;
    }

    /**
     * Calculates the matrix's inverse and returns it.
     *
     * @return The inverse of this matrix
     */
    public @Dimensionless Matrix4x4 getInverse(@Dimensionless Matrix4x4 this) {
        final @Dimensionless float determinant = getDeterminant();
        if (determinant != ((@Dimensionless int) (0))) {
            final @Dimensionless Matrix4x4 tempMatrix = new @Dimensionless Matrix4x4();
            final @Dimensionless float t00 = determinant3x3(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))),
                    mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))));
            final @Dimensionless float t01 = -determinant3x3(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))),
                    mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))));
            final @Dimensionless float t02 = determinant3x3(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))),
                    mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))));
            final @Dimensionless float t03 = -determinant3x3(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))),
                    mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))));
            final @Dimensionless float t10 = -determinant3x3(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))),
                    mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))));
            final @Dimensionless float t11 = determinant3x3(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))),
                    mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))));
            final @Dimensionless float t12 = -determinant3x3(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))),
                    mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))));
            final @Dimensionless float t13 = determinant3x3(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))),
                    mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))));
            final @Dimensionless float t20 = determinant3x3(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))),
                    mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))));
            final @Dimensionless float t21 = -determinant3x3(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))),
                    mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))));
            final @Dimensionless float t22 = determinant3x3(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))),
                    mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))));
            final @Dimensionless float t23 = -determinant3x3(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))),
                    mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))));
            final @Dimensionless float t30 = -determinant3x3(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))),
                    mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))));
            final @Dimensionless float t31 = determinant3x3(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))),
                    mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))));
            final @Dimensionless float t32 = -determinant3x3(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))),
                    mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))));
            final @Dimensionless float t33 = determinant3x3(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))),
                    mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))));
            final @Dimensionless float determinant_inv = ((@Dimensionless int) (1)) / determinant;
            tempMatrix.set(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), t00 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (1)), ((@Dimensionless int) (1)), t11 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (2)), ((@Dimensionless int) (2)), t22 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (3)), ((@Dimensionless int) (3)), t33 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (1)), ((@Dimensionless int) (0)), t10 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (0)), ((@Dimensionless int) (1)), t01 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (0)), ((@Dimensionless int) (2)), t02 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (2)), ((@Dimensionless int) (0)), t20 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (2)), ((@Dimensionless int) (1)), t21 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (1)), ((@Dimensionless int) (2)), t12 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (3)), ((@Dimensionless int) (0)), t30 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (0)), ((@Dimensionless int) (3)), t03 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (3)), ((@Dimensionless int) (1)), t31 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (1)), ((@Dimensionless int) (3)), t13 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (2)), ((@Dimensionless int) (3)), t23 * determinant_inv);
            tempMatrix.set(((@Dimensionless int) (3)), ((@Dimensionless int) (2)), t32 * determinant_inv);
            return tempMatrix;
        } else {
            return null;
        }
    }

    /**
     * Calculates the trace of this matrix and returns it.
     *
     * @return The trace of this matrix
     */
    public @Dimensionless float getTrace(@Dimensionless Matrix4x4 this) {
        return mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) + mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) + mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) + mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3)));
    }

    /**
     * Creates a matrix where each value is the absolute of the value for this matrix and returns it.
     *
     * @return A new matrix which is the absolute version of this one
     */
    public @Dimensionless Matrix4x4 getAbsoluteMatrix(@Dimensionless Matrix4x4 this) {
        return new @Dimensionless Matrix4x4(
                Math.abs(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0)))), Math.abs(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1)))), Math.abs(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2)))), Math.abs(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3)))),
                Math.abs(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0)))), Math.abs(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1)))), Math.abs(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2)))), Math.abs(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3)))),
                Math.abs(mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0)))), Math.abs(mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1)))), Math.abs(mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2)))), Math.abs(mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3)))),
                Math.abs(mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0)))), Math.abs(mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1)))), Math.abs(mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2)))), Math.abs(mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3)))));
    }

    /**
     * Adds the matrix to this one. Doesn't create a new matrix.
     *
     * @param matrix The matrix to add
     * @return This matrix for chained calls
     */
    public @Dimensionless Matrix4x4 add(@Dimensionless Matrix4x4 this, @Dimensionless Matrix4x4 matrix) {
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))));
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (3)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3))) + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (3)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))) + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))) + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (3)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))) + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))));
        mRows[((@Dimensionless int) (3))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))) + matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (3))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))) + matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (3))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))) + matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))));
        mRows[((@Dimensionless int) (3))].set(((@Dimensionless int) (3)), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))) + matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))));
        return this;
    }

    /**
     * Subtracts the matrix to this one. Doesn't create a new matrix.
     *
     * @param matrix The matrix to subtract
     * @return This matrix for chained calls
     */
    public @Dimensionless Matrix4x4 subtract(@Dimensionless Matrix4x4 this, @Dimensionless Matrix4x4 matrix) {
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))));
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (3)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (3)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) - matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) - matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (3)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))) - matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))));
        mRows[((@Dimensionless int) (3))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (3))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))) - matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (3))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))) - matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))));
        mRows[((@Dimensionless int) (3))].set(((@Dimensionless int) (3)), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))) - matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))));
        return this;
    }

    /**
     * Multiplies the matrix to this one. Doesn't create a new matrix.
     *
     * @param value The value to multiply
     * @return This matrix for chained calls
     */
    public @Dimensionless Matrix4x4 multiply(@Dimensionless Matrix4x4 this, @Dimensionless float value) {
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) * value);
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * value);
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) * value);
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (3)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (3))) * value);
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) * value);
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) * value);
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) * value);
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (3)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (3))) * value);
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))) * value);
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) * value);
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) * value);
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (3)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (3))) * value);
        mRows[((@Dimensionless int) (3))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (0))) * value);
        mRows[((@Dimensionless int) (3))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (1))) * value);
        mRows[((@Dimensionless int) (3))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (2))) * value);
        mRows[((@Dimensionless int) (3))].set(((@Dimensionless int) (3)), mRows[((@Dimensionless int) (3))].get(((@Dimensionless int) (3))) * value);
        return this;
    }

    @Override
    public @Dimensionless int hashCode(@Dimensionless Matrix4x4 this) {
        @Dimensionless
        int hash = ((@Dimensionless int) (7));
        hash = ((@Dimensionless int) (83)) * hash + Arrays.deepHashCode(mRows);
        return hash;
    }

    @Override
    public @Dimensionless boolean equals(@Dimensionless Matrix4x4 this, @Dimensionless Object obj) {
        if (!(obj instanceof Matrix4x4)) {
            return false;
        }
        final @Dimensionless Matrix4x4 other = (@Dimensionless Matrix4x4) obj;
        return Arrays.deepEquals(mRows, other.mRows);
    }

    /**
     * Returns a new identity matrix.
     *
     * @return A new identity matrix
     */
    public static @Dimensionless Matrix4x4 identity() {
        return new @Dimensionless Matrix4x4(
                ((@Dimensionless int) (1)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless int) (1)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (1)), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (1)));
    }

    /**
     * Adds the two matrices and returns the result as a new matrix.
     *
     * @param matrix1 The first matrix
     * @param matrix2 The second matrix
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix4x4 add(@Dimensionless Matrix4x4 matrix1, @Dimensionless Matrix4x4 matrix2) {
        return new @Dimensionless Matrix4x4(
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) + matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) + matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) + matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))), matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))) + matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))),
                matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) + matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) + matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) + matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))), matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))) + matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))),
                matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) + matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) + matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) + matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))), matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))) + matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))),
                matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))) + matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))) + matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))) + matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))), matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))) + matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))));
    }

    /**
     * Subtracts the two matrices and returns the result as a new matrix.
     *
     * @param matrix1 The first matrix
     * @param matrix2 The second matrix
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix4x4 subtract(@Dimensionless Matrix4x4 matrix1, @Dimensionless Matrix4x4 matrix2) {
        return new @Dimensionless Matrix4x4(
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) - matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) - matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) - matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))), matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))) - matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))),
                matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) - matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) - matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) - matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))), matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))) - matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))),
                matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) - matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) - matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) - matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))), matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))) - matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))),
                matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))) - matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))) - matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))) - matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))), matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))) - matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))));
    }

    /**
     * Negates the matrix and returns the result as a new matrix.
     *
     * @param matrix The matrix to negate
     * @return The negated version of the matrix
     */
    public static @Dimensionless Matrix4x4 negate(@Dimensionless Matrix4x4 matrix) {
        return new @Dimensionless Matrix4x4(
                -matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), -matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))), -matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))), -matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))),
                -matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))), -matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))), -matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))), -matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))),
                -matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))), -matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))), -matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))), -matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))),
                -matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))), -matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))), -matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))), -matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))));
    }

    /**
     * Multiplies a float value by a matrix and returns the result as a new matrix.
     *
     * @param value The value
     * @param matrix The matrix
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix4x4 multiply(@Dimensionless float value, @Dimensionless Matrix4x4 matrix) {
        return multiply(matrix, value);
    }

    /**
     * Multiplies a matrix by a float value and returns the result as a new matrix.
     *
     * @param matrix The matrix
     * @param value The value
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix4x4 multiply(@Dimensionless Matrix4x4 matrix, @Dimensionless float value) {
        return new @Dimensionless Matrix4x4(
                matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * value, matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * value, matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) * value, matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))) * value,
                matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * value, matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * value, matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) * value, matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))) * value,
                matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) * value, matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) * value, matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) * value, matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))) * value,
                matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))) * value, matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))) * value, matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))) * value, matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))) * value);
    }

    /**
     * Multiplies the two matrices and returns the result as a new matrix.
     *
     * @param matrix1 The first matrix
     * @param matrix2 The second matrix
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix4x4 multiply(@Dimensionless Matrix4x4 matrix1, @Dimensionless Matrix4x4 matrix2) {
        final @Dimensionless float m00 = matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0)))
                + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0)));
        final @Dimensionless float m01 = matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0)))
                + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0)));
        final @Dimensionless float m02 = matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0)))
                + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0)));
        final @Dimensionless float m03 = matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0)))
                + matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0)));
        final @Dimensionless float m10 = matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1)))
                + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1)));
        final @Dimensionless float m11 = matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1)))
                + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1)));
        final @Dimensionless float m12 = matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1)))
                + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1)));
        final @Dimensionless float m13 = matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) + matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1)))
                + matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) + matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1)));
        final @Dimensionless float m20 = matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2)))
                + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2)));
        final @Dimensionless float m21 = matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2)))
                + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2)));
        final @Dimensionless float m22 = matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2)))
                + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2)));
        final @Dimensionless float m23 = matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) + matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2)))
                + matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) + matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2)));
        final @Dimensionless float m30 = matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))) + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3)))
                + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))) + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3)));
        final @Dimensionless float m31 = matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))) + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3)))
                + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))) + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3)));
        final @Dimensionless float m32 = matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))) + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3)))
                + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))) + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3)));
        final @Dimensionless float m33 = matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))) + matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3)))
                + matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))) + matrix1.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))) * matrix2.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3)));
        return new @Dimensionless Matrix4x4(
                m00, m10, m20, m30,
                m01, m11, m21, m31,
                m02, m12, m22, m32,
                m03, m13, m23, m33);
    }

    /**
     * Multiplies a matrix by a vector and returns the result as a new vector.
     *
     * @param matrix The matrix
     * @param vector The vector
     * @return The resulting matrix
     */
    public static @Dimensionless Vector4 multiply(@Dimensionless Matrix4x4 matrix, @Dimensionless Vector4 vector) {
        return new @Dimensionless Vector4(
                matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * vector.getX() + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * vector.getY() + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) * vector.getZ() + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (3))) * vector.getW(),
                matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * vector.getX() + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * vector.getY() + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) * vector.getZ() + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (3))) * vector.getW(),
                matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) * vector.getX() + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) * vector.getY() + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) * vector.getZ() + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (3))) * vector.getW(),
                matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (0))) * vector.getX() + matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (1))) * vector.getY() + matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (2))) * vector.getZ() + matrix.get(((@Dimensionless int) (3)), ((@Dimensionless int) (3))) * vector.getW());
    }

    // Returns the determinant of a 3x3 matrix
    private static @Dimensionless float determinant3x3(@Dimensionless float t00, @Dimensionless float t01, @Dimensionless float t02,
                                        @Dimensionless
                                        float t10, @Dimensionless float t11, @Dimensionless float t12,
                                        @Dimensionless
                                        float t20, @Dimensionless float t21, @Dimensionless float t22) {
        return t00 * (t11 * t22 - t12 * t21) + t01 * (t12 * t20 - t10 * t22) + t02 * (t10 * t21 - t11 * t20);
    }
}
