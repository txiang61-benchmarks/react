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
 *
 */
public class Matrix2x2 {
    /**
     * The first row of the matrix. Value of 0
     */
    public static final @Dimensionless int FIRST_ROW = ((@Dimensionless int) (0));
    /**
     * The second row of the matrix. Value of 1
     */
    public static final @Dimensionless int SECOND_ROW = ((@Dimensionless int) (1));
    /**
     * The first column of the matrix. Value of 0
     */
    public static final @Dimensionless int FIRST_COLUMN = ((@Dimensionless int) (0));
    /**
     * The second column of the matrix. Value of 1
     */
    public static final @Dimensionless int SECOND_COLUMN = ((@Dimensionless int) (1));
    private final @Dimensionless Vector2 @Dimensionless [] mRows = new @Dimensionless Vector2 @Dimensionless [] {
            new @Dimensionless Vector2(),
            new @Dimensionless Vector2(),
    };

    /**
     * Default constructor. All values are 0.
     */
    public Matrix2x2() {
        this(((@Dimensionless int) (0)));
    }

    /**
     * Constructs a new 2x2 matrix with all values set to the provided one.
     *
     * @param value The value to use
     */
    public Matrix2x2(@Dimensionless float value) {
        this(value, value, value, value);
    }

    /**
     * Copy constructor.
     *
     * @param matrix The matrix to copy
     */
    public Matrix2x2(@Dimensionless Matrix2x2 matrix) {
        this(
                matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))),
                matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))));
    }

    /**
     * Construct a new matrix from all the values.
     *
     * @param a1 The value for 0,0
     * @param a2 The value for 0,1
     * @param b1 The value for 1,0
     * @param b2 The value for 1,1
     */
    public Matrix2x2(float a1, float a2,
                     float b1, float b2) {
        setAllValues(a1, a2, b1, b2);
    }

    /**
     * Sets all the values to the provided ones.
     *
     * @param a1 The value for 0,0
     * @param a2 The value for 0,1
     * @param b1 The value for 1,0
     * @param b2 The value for 1,1
     */
    public final void setAllValues(@Dimensionless Matrix2x2 this, @Dimensionless float a1, @Dimensionless float a2,
                                   @Dimensionless
                                   float b1, @Dimensionless float b2) {
        mRows[((@Dimensionless int) (0))].setAllValues(a1, a2);
        mRows[((@Dimensionless int) (1))].setAllValues(b1, b2);
    }

    /**
     * Sets the values of this matrix to those of the provided matrix.
     *
     * @param matrix The matrix to copy the values from
     * @return This matrix for chained calls
     */
    public Matrix2x2 set(@Dimensionless Matrix2x2 this, Matrix2x2 matrix) {
        setAllValues(
                matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))),
                matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))), matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))));
        return this;
    }

    /**
     * Gets the desired row as a vector2.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}
     * @return The vector2 for the row
     */
    public @Dimensionless Vector2 get(@Dimensionless Matrix2x2 this, @Dimensionless int row) {
        return mRows[row];
    }

    /**
     * Gets the desired value at the row and column.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}
     * @param col The column number, either {@link #FIRST_COLUMN}, {@link #SECOND_COLUMN}
     * @return The value at the row and column
     */
    public @Dimensionless float get(@Dimensionless Matrix2x2 this, @Dimensionless int row, @Dimensionless int col) {
        return mRows[row].get(col);
    }

    /**
     * Gets the desired column as a vector2.
     *
     * @param col The column number, either {@link #FIRST_COLUMN}, {@link #SECOND_COLUMN}
     * @return The column as a vector2
     */
    public @Dimensionless Vector2 getColumn(@Dimensionless Matrix2x2 this, @Dimensionless int col) {
        return new @Dimensionless Vector2(mRows[((@Dimensionless int) (0))].get(col), mRows[((@Dimensionless int) (1))].get(col));
    }

    /**
     * Gets the desired row as a vector2.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}
     * @return The vector2 for the row
     */
    public @Dimensionless Vector2 getRow(@Dimensionless Matrix2x2 this, @Dimensionless int row) {
        return mRows[row];
    }

    /**
     * Sets the value at the row and column to the desired value.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}
     * @param col The column number, either {@link #FIRST_COLUMN}, {@link #SECOND_COLUMN}
     * @param value The value to set at the row and column
     */
    public void set(@Dimensionless Matrix2x2 this, @Dimensionless int row, @Dimensionless int col, @Dimensionless float value) {
        mRows[row].set(col, value);
    }

    /**
     * Sets all the values at zero.
     */
    public void setToZero(@Dimensionless Matrix2x2 this) {
        mRows[((@Dimensionless int) (0))].setToZero();
        mRows[((@Dimensionless int) (1))].setToZero();
    }

    /**
     * Sets all the matrix values to identity.
     */
    public void setToIdentity(@Dimensionless Matrix2x2 this) {
        mRows[((@Dimensionless int) (0))].setAllValues(((@Dimensionless int) (1)), ((@Dimensionless int) (0)));
        mRows[((@Dimensionless int) (1))].setAllValues(((@Dimensionless int) (0)), ((@Dimensionless int) (1)));
    }

    /**
     * Transposes the matrix to a new matrix and returns it.
     *
     * @return The new matrix which is the transposed version of this one
     */
    public @Dimensionless Matrix2x2 getTranspose(@Dimensionless Matrix2x2 this) {
        return new @Dimensionless Matrix2x2(
                mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))),
                mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))));
    }

    /**
     * Calculates the determinant of this matrix and returns it.
     *
     * @return The determinant of this matrix
     */
    public @Dimensionless float getDeterminant(@Dimensionless Matrix2x2 this) {
        return mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) - mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0)));
    }

    /**
     * Calculates the matrix's inverse and returns it.
     *
     * @return The inverse of this matrix
     */
    public Matrix2x2 getInverse(@Dimensionless Matrix2x2 this) {
        final @Dimensionless float determinant = getDeterminant();
        if (Math.abs(determinant) <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalStateException("Determinant of matrix cannot be zero");
        }
        final @Dimensionless float invDeterminant = ((@Dimensionless int) (1)) / determinant;
        final @Dimensionless Matrix2x2 tempMatrix = new @Dimensionless Matrix2x2(
                mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))), -mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))),
                -mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))));
        return tempMatrix.multiply(invDeterminant);
    }

    /**
     * Calculates the trace of this matrix and returns it.
     *
     * @return The trace of this matrix
     */
    public @Dimensionless float getTrace(@Dimensionless Matrix2x2 this) {
        return mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) + mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1)));
    }

    /**
     * Creates a matrix where each value is the absolute of the value for this matrix and returns it.
     *
     * @return A new matrix which is the absolute version of this one
     */
    public @Dimensionless Matrix2x2 getAbsoluteMatrix(@Dimensionless Matrix2x2 this) {
        return new @Dimensionless Matrix2x2(
                Math.abs(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0)))), Math.abs(mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1)))),
                Math.abs(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0)))), Math.abs(mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1)))));
    }

    /**
     * Adds the matrix to this one. Doesn't create a new matrix.
     *
     * @param matrix The matrix to add
     * @return This matrix for chained calls
     */
    public @Dimensionless Matrix2x2 add(@Dimensionless Matrix2x2 this, @Dimensionless Matrix2x2 matrix) {
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))));
        return this;
    }

    /**
     * Subtracts the matrix to this one. Doesn't create a new matrix.
     *
     * @param matrix The matrix to subtract
     * @return This matrix for chained calls
     */
    public @Dimensionless Matrix2x2 subtract(@Dimensionless Matrix2x2 this, @Dimensionless Matrix2x2 matrix) {
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))));
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))));
        return this;
    }

    /**
     * Multiplies the matrix to this one. Doesn't create a new matrix.
     *
     * @param value The value to multiply
     * @return This matrix for chained calls
     */
    public @Dimensionless Matrix2x2 multiply(@Dimensionless Matrix2x2 this, @Dimensionless float value) {
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (0))) * value);
        mRows[((@Dimensionless int) (0))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (0))].get(((@Dimensionless int) (1))) * value);
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (0)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (0))) * value);
        mRows[((@Dimensionless int) (1))].set(((@Dimensionless int) (1)), mRows[((@Dimensionless int) (1))].get(((@Dimensionless int) (1))) * value);
        return this;
    }

    @Override
    public @Dimensionless int hashCode(@Dimensionless Matrix2x2 this) {
        @Dimensionless
        int hash = ((@Dimensionless int) (7));
        hash = ((@Dimensionless int) (83)) * hash + Arrays.deepHashCode(mRows);
        return hash;
    }

    @Override
    public @Dimensionless boolean equals(@Dimensionless Matrix2x2 this, @Dimensionless Object obj) {
        if (!(obj instanceof Matrix2x2)) {
            return false;
        }
        final @Dimensionless Matrix2x2 other = (@Dimensionless Matrix2x2) obj;
        return Arrays.deepEquals(mRows, other.mRows);
    }

    /**
     * Returns a new identity matrix.
     *
     * @return A new identity matrix
     */
    public static @Dimensionless Matrix2x2 identity() {
        return new @Dimensionless Matrix2x2(
                ((@Dimensionless int) (1)), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless int) (1)));
    }

    /**
     * Adds the two matrices and returns the result as a new matrix.
     *
     * @param matrix1 The first matrix
     * @param matrix2 The second matrix
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix2x2 add(@Dimensionless Matrix2x2 matrix1, @Dimensionless Matrix2x2 matrix2) {
        return new @Dimensionless Matrix2x2(
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) + matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) + matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) + matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) + matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))));
    }

    /**
     * Subtracts the two matrices and returns the result as a new matrix.
     *
     * @param matrix1 The first matrix
     * @param matrix2 The second matrix
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix2x2 subtract(@Dimensionless Matrix2x2 matrix1, @Dimensionless Matrix2x2 matrix2) {
        return new @Dimensionless Matrix2x2(
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) - matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) - matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) - matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))), matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) - matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))));
    }

    /**
     * Negates the matrix and returns the result as a new matrix.
     *
     * @param matrix The matrix to negate
     * @return The negated version of the matrix
     */
    public static @Dimensionless Matrix2x2 negate(@Dimensionless Matrix2x2 matrix) {
        return new @Dimensionless Matrix2x2(
                -matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))), -matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))),
                -matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))), -matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))));
    }

    /**
     * Multiplies a float value by a matrix and returns the result as a new matrix.
     *
     * @param value The value
     * @param matrix The matrix
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix2x2 multiply(@Dimensionless float value, @Dimensionless Matrix2x2 matrix) {
        return multiply(matrix, value);
    }

    /**
     * Multiplies a matrix by a float value and returns the result as a new matrix.
     *
     * @param matrix The matrix
     * @param value The value
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix2x2 multiply(@Dimensionless Matrix2x2 matrix, @Dimensionless float value) {
        return new @Dimensionless Matrix2x2(
                matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * value, matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * value,
                matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * value, matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * value);
    }

    /**
     * Multiplies the two matrices and returns the result as a new matrix.
     *
     * @param matrix1 The first matrix
     * @param matrix2 The second matrix
     * @return The resulting matrix
     */
    public static @Dimensionless Matrix2x2 multiply(@Dimensionless Matrix2x2 matrix1, @Dimensionless Matrix2x2 matrix2) {
        return new @Dimensionless Matrix2x2(
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))),
                matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) + matrix1.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))),
                matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))),
                matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * matrix2.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) + matrix1.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * matrix2.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1)))
        );
    }

    /**
     * Multiplies a matrix by a vector and returns the result as a new vector.
     *
     * @param matrix The matrix
     * @param vector The vector
     * @return The resulting matrix
     */
    public static Vector2 multiply(Matrix2x2 matrix, Vector2 vector) {
        return new @Dimensionless Vector2(
                matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) * vector.getX() + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) * vector.getY(),
                matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) * vector.getX() + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) * vector.getY());
    }
}
