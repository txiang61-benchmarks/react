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

import com.flowpowered.react.ReactDefaults;

/**
 * Represents a 3x3 matrix.
 */
public class Matrix3x3 {
    /**
     * The first row of the matrix. Value of 0
     */
    public static final @Dimensionless int FIRST_ROW = ((@Dimensionless int) (0));
    /**
     * The second row of the matrix. Value of 1
     */
    public static final @Dimensionless int SECOND_ROW = ((@Dimensionless int) (1));
    /**
     * The third row of the matrix. Value of 2
     */
    public static final @Dimensionless int THIRD_ROW = ((@Dimensionless int) (2));
    /**
     * The first column of the matrix. Value of 0
     */
    public static final @Dimensionless int FIRST_COLUMN = ((@Dimensionless int) (0));
    /**
     * The second column of the matrix. Value of 1
     */
    public static final @Dimensionless int SECOND_COLUMN = ((@Dimensionless int) (1));
    /**
     * The third column of the matrix. Value of 2
     */
    public static final @Dimensionless int THIRD_COLUMN = ((@Dimensionless int) (2));
    private final @Dimensionless Vector3 @Dimensionless [] mRows = new @Dimensionless Vector3 @Dimensionless [] {
            new @Dimensionless Vector3(),
            new @Dimensionless Vector3(),
            new @Dimensionless Vector3()
    };

    /**
     * Default constructor. All values are 0.
     */
    public Matrix3x3() {
        this(((@Dimensionless int) (0)));
    }

    /**
     * Constructs a new 3x3 matrix with all values set to the provided one.
     *
     * @param value The value to use
     */
    public Matrix3x3(@Dimensionless float value) {
        this(value, value, value, value, value, value, value, value, value);
    }

    /**
     * Copy constructor.
     *
     * @param matrix The matrix to copy
     */
    public Matrix3x3(@Dimensionless Matrix3x3 matrix) {
        this(
                matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))),
                matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))),
                matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))));
    }

    /**
     * Construct a new matrix from all the values.
     *
     * @param a1 The value for 0,0
     * @param a2 The value for 0,1
     * @param a3 The value for 0,2
     * @param b1 The value for 1,0
     * @param b2 The value for 1,1
     * @param b3 The value for 1,2
     * @param c1 The value for 2,0
     * @param c2 The value for 2,1
     * @param c3 The value for 2,2
     */
    public Matrix3x3(float a1, float a2, float a3,
                     float b1, float b2, float b3,
                     float c1, float c2, float c3) {
        setAllValues(a1, a2, a3, b1, b2, b3, c1, c2, c3);
    }

    /**
     * Sets all the values to the provided ones.
     *
     * @param a1 The value for 0,0
     * @param a2 The value for 0,1
     * @param a3 The value for 0,2
     * @param b1 The value for 1,0
     * @param b2 The value for 1,1
     * @param b3 The value for 1,2
     * @param c1 The value for 2,0
     * @param c2 The value for 2,1
     * @param c3 The value for 2,2
     */
    public final void setAllValues(@Dimensionless Matrix3x3 this, @Dimensionless float a1, @Dimensionless float a2, @Dimensionless float a3,
                                   @Dimensionless
                                   float b1, @Dimensionless float b2, @Dimensionless float b3,
                                   @Dimensionless
                                   float c1, @Dimensionless float c2, @Dimensionless float c3) {
        mRows[((@Dimensionless int) (0))].setAllValues(a1, a2, a3);
        mRows[((@Dimensionless int) (1))].setAllValues(b1, b2, b3);
        mRows[((@Dimensionless int) (2))].setAllValues(c1, c2, c3);
    }

    /**
     * Sets the values of this matrix to those of the provided matrix.
     *
     * @param matrix The matrix to copy the values from
     * @return This matrix for chained calls
     */
    public @Dimensionless Matrix3x3 set(@Dimensionless Matrix3x3 this, @Dimensionless Matrix3x3 matrix) {
        setAllValues(
                matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))),
                matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))),
                matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))), matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))));
        return this;
    }

    /**
     * Gets the desired row as a vector3.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}, {@link #THIRD_ROW}
     * @return The vector3 for the row
     */
    public @Dimensionless Vector3 get(@Dimensionless Matrix3x3 this, @Dimensionless int row) {
        return mRows[row];
    }

    /**
     * Gets the desired value at the row and column.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}, {@link #THIRD_ROW}
     * @param col The column number, either {@link #FIRST_COLUMN}, {@link #SECOND_COLUMN}, {@link #THIRD_COLUMN}
     * @return The value at the row and column
     */
    public float get(@Dimensionless Matrix3x3 this, int row, int col) {
        return mRows[row].get(col);
    }

    /**
     * Gets the desired column as a vector3.
     *
     * @param col The column number, either {@link #FIRST_COLUMN}, {@link #SECOND_COLUMN}, {@link #THIRD_COLUMN}
     * @return The column as a vector3
     */
    public @Dimensionless Vector3 getColumn(@Dimensionless Matrix3x3 this, @Dimensionless int col) {
        return new @Dimensionless Vector3(mRows[((@Dimensionless int) (0))].get(col), mRows[((@Dimensionless int) (1))].get(col), mRows[((@Dimensionless int) (2))].get(col));
    }

    /**
     * Gets the desired row as a vector3.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}, {@link #THIRD_ROW}
     * @return The vector3 for the row
     */
    public @Dimensionless Vector3 getRow(@Dimensionless Matrix3x3 this, @Dimensionless int row) {
        return mRows[row];
    }

    /**
     * Sets the value at the row and column to the desired value.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}, {@link #THIRD_ROW}
     * @param col The column number, either {@link #FIRST_COLUMN}, {@link #SECOND_COLUMN}, {@link #THIRD_COLUMN}
     * @param value The value to set at the row and column
     */
    public void set(@Dimensionless Matrix3x3 this, @Dimensionless int row, @Dimensionless int col, @Dimensionless float value) {
        mRows[row].set(col, value);
    }

    /**
     * Sets all the values at zero.
     */
    public void setToZero(@Dimensionless Matrix3x3 this) {
        mRows[((@Dimensionless int) (0))].setToZero();
        mRows[((@Dimensionless int) (1))].setToZero();
        mRows[((@Dimensionless int) (2))].setToZero();
    }

    /**
     * Sets all the matrix values to identity.
     */
    public void setToIdentity(@Dimensionless Matrix3x3 this) {
        mRows[((@Dimensionless int) (0))].setAllValues(((@Dimensionless int) (1)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        mRows[((@Dimensionless int) (1))].setAllValues(((@Dimensionless int) (0)), ((@Dimensionless int) (1)), ((@Dimensionless int) (0)));
        mRows[((@Dimensionless int) (2))].setAllValues(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (1)));
    }

    /**
     * Transposes the matrix to a new matrix and returns it.
     *
     * @return The new matrix which is the transposed version of this one
     */
    public @Dimensionless Matrix3x3 getTranspose(@Dimensionless Matrix3x3 this) {
        return new @Dimensionless Matrix3x3(
                mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))),
                mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))),
                mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))));
    }

    /**
     * Calculates the determinant of this matrix and returns it.
     *
     * @return The determinant of this matrix
     */
    public @Dimensionless float getDeterminant(@Dimensionless Matrix3x3 this) {
        return mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) * (mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) - mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))))
                - mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * (mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) - mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))))
                + mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) * (mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) - mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))));
    }

    /**
     * Calculates the matrix's inverse and returns it.
     *
     * @return The inverse of this matrix
     */
    public @Dimensionless Matrix3x3 getInverse(@Dimensionless Matrix3x3 this) {
        final @Dimensionless float determinant = getDeterminant();
        if (Math.abs(determinant) <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalStateException("Determinant of matrix cannot be zero");
        }
        final @Dimensionless float invDeterminant = ((@Dimensionless int) (1)) / determinant;
        final @Dimensionless Matrix3x3 tempMatrix = new @Dimensionless Matrix3x3(
                (mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) - mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2)))),
                -(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) - mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2)))),
                (mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) - mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1)))),
                -(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) - mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2)))),
                (mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) - mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2)))),
                -(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) - mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2)))),
                (mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) - mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1)))),
                -(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) - mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1)))),
                (mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) - mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0)))));
        return Matrix3x3.multiply(invDeterminant, tempMatrix);
    }

    /**
     * Calculates the trace of this matrix and returns it.
     *
     * @return The trace of this matrix
     */
    public float getTrace(@Dimensionless Matrix3x3 this) {
        return mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) + mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) + mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2)));
    }

    /**
     * Creates a matrix where each value is the absolute of the value for this matrix and returns it.
     *
     * @return A new matrix which is the absolute version of this one
     */
    public @Dimensionless Matrix3x3 getAbsoluteMatrix(@Dimensionless Matrix3x3 this) {
        return new @Dimensionless Matrix3x3(
                Math.abs(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0)))), Math.abs(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1)))), Math.abs(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2)))),
                Math.abs(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0)))), Math.abs(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1)))), Math.abs(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2)))),
                Math.abs(mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0)))), Math.abs(mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1)))), Math.abs(mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2)))));
    }

    /**
     * Adds the matrix to this one. Doesn't create a new matrix.
     *
     * @param matrix The matrix to add
     * @return This matrix for chained calls
     */
    public @Dimensionless Matrix3x3 add(@Dimensionless Matrix3x3 this, @Dimensionless Matrix3x3 matrix) {
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))) + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))));
        return this;
    }

    /**
     * Subtracts the matrix to this one. Doesn't create a new matrix.
     *
     * @param matrix The matrix to subtract
     * @return This matrix for chained calls
     */
    public @Dimensionless Matrix3x3 subtract(@Dimensionless Matrix3x3 this, @Dimensionless Matrix3x3 matrix) {
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) - matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) - matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))));
        return this;
    }

    /**
     * Multiplies the matrix to this one. Doesn't create a new matrix.
     *
     * @param value The value to multiply
     * @return This matrix for chained calls
     */
    public @Dimensionless Matrix3x3 multiply(@Dimensionless Matrix3x3 this, @Dimensionless float value) {
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) * value);
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * value);
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (2))) * value);
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) * value);
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) * value);
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (2))) * value);
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (0))) * value);
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (1))) * value);
        mRows[((@Dimensionless int) (2))].set(((@Dimensionless int) (2)), mRows[((@Dimensionless int) (2))].get(((@Dimensionless int) (2))) * value);
        return this;
    }

    @Override
    public @Dimensionless int hashCode(@Dimensionless Matrix3x3 this) {
        @Dimensionless
        int hash = ((@Dimensionless int) (7));
        hash = ((@Dimensionless int) (83)) * hash + Arrays.deepHashCode(mRows);
        return hash;
    }

    @Override
    public @Dimensionless boolean equals(@Dimensionless Matrix3x3 this, @Dimensionless Object obj) {
        if (!(obj instanceof Matrix3x3)) {
            return false;
        }
        final @Dimensionless Matrix3x3 other = (@Dimensionless Matrix3x3) obj;
        return Arrays.deepEquals(mRows, other.mRows);
    }

    /**
     * Returns a new identity matrix.
     *
     * @return A new identity matrix
     */
    public static @Dimensionless Matrix3x3 identity() {
        return new @Dimensionless Matrix3x3(
                ((@Dimensionless int) (1)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless int) (1)), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (1)));
    }

    /**
     * Adds the two matrices and returns the result as a new matrix.
     *
     * @param matrix1 The first matrix
     * @param matrix2 The second matrix
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix3x3 add(@Dimensionless Matrix3x3 matrix1, @Dimensionless Matrix3x3 matrix2) {
        return new @Dimensionless Matrix3x3(
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) + matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) + matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) + matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))), matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) + matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))),
                matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) + matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))), matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) + matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))),
                matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) + matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) + matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) + matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))));
    }

    /**
     * Subtracts the two matrices and returns the result as a new matrix.
     *
     * @param matrix1 The first matrix
     * @param matrix2 The second matrix
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix3x3 subtract(@Dimensionless Matrix3x3 matrix1, @Dimensionless Matrix3x3 matrix2) {
        return new @Dimensionless Matrix3x3(
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) - matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) - matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) - matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))), matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) - matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))),
                matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) - matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))), matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) - matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))),
                matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) - matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) - matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) - matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))));
    }

    /**
     * Negates the matrix and returns the result as a new matrix.
     *
     * @param matrix The matrix to negate
     * @return The negated version of the matrix
     */
    public static @Dimensionless Matrix3x3 negate(@Dimensionless Matrix3x3 matrix) {
        return new @Dimensionless Matrix3x3(
                -matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), -matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))), -matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))),
                -matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))), -matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))), -matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))),
                -matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))), -matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))), -matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))));
    }

    /**
     * Multiplies a float value by a matrix and returns the result as a new matrix.
     *
     * @param value The value
     * @param matrix The matrix
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix3x3 multiply(@Dimensionless float value, @Dimensionless Matrix3x3 matrix) {
        return multiply(matrix, value);
    }

    /**
     * Multiplies a matrix by a float value and returns the result as a new matrix.
     *
     * @param matrix The matrix
     * @param value The value
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix3x3 multiply(@Dimensionless Matrix3x3 matrix, @Dimensionless float value) {
        return new @Dimensionless Matrix3x3(
                matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * value, matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * value, matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) * value,
                matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * value, matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * value, matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) * value,
                matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) * value, matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) * value, matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) * value);
    }

    /**
     * Multiplies the two matrices and returns the result as a new matrix.
     *
     * @param matrix1 The first matrix
     * @param matrix2 The second matrix
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix3x3 multiply(@Dimensionless Matrix3x3 matrix1, @Dimensionless Matrix3x3 matrix2) {
        return new @Dimensionless Matrix3x3(
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0)))
                        + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1)))
                + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2)))
                        + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))), matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0)))
                + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))),
                matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1)))
                        + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))), matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2)))
                + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))),
                matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0)))
                        + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1)))
                + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2)))
                        + matrix1.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) * matrix2.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2)))
        );
    }

    /**
     * Multiplies a matrix by a vector and returns the result as a new vector.
     *
     * @param matrix The matrix
     * @param vector The vector
     * @return The resulting matrix
     */
    public static Vector3 multiply(Matrix3x3 matrix, Vector3 vector) {
        return new @Dimensionless Vector3(
                matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * vector.getX() + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * vector.getY() + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) * vector.getZ(),
                matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * vector.getX() + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * vector.getY() + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) * vector.getZ(),
                matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) * vector.getX() + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) * vector.getY() + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) * vector.getZ());
    }

    /**
     * Returns a skew-symmetric matrix using a given vector that can be used to compute cross product with another vector using matrix multiplication,
     *
     * @return A skew-symmetric matrix for cross product
     */
    public static @Dimensionless Matrix3x3 computeSkewSymmetricMatrixForCrossProduct(@Dimensionless Vector3 vector) {
        return new @Dimensionless Matrix3x3(
                ((@Dimensionless int) (0)), -vector.getZ(), vector.getY(),
                vector.getZ(), ((@Dimensionless int) (0)), -vector.getX(),
                -vector.getY(), vector.getX(), ((@Dimensionless int) (0)));
    }
}
