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
package com.flowpowered.react;
import units.qual.Dimensionless;

/**
 * This class contains static utilities. It was added for the port to implement some C++ code in Java.
 */
@Dimensionless
public class Utilities {
    /**
     * Returns the index of an object in an array, or -1 if it can't be found.
     *
     * @param array The array to search
     * @param object The object to look for
     * @return The index, or -1 if the object wasn't found
     */
    public static @Dimensionless int indexOf(@Dimensionless Object @Dimensionless [] array, @Dimensionless Object object) {
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < array.length; i++) {
            if (object.equals(array[i])) {
                return i;
            }
        }
        return ((@Dimensionless int) (-1));
    }

    /**
     * Represents a pair of 32 bit integers.
     */
    @Dimensionless
    public static class IntPair {
        private @Dimensionless int first;
        private @Dimensionless int second;

        /**
         * Constructs a new int pair with both integers being 0.
         */
        public IntPair() {
            this(((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        }

        /**
         * Constructs a new int pair with the desired value for each member.
         *
         * @param first The value of the first member
         * @param second The value of the second member
         */
        public IntPair(Utilities.@Dimensionless IntPair this, @Dimensionless int first, @Dimensionless int second) {
            this.first = first;
            this.second = second;
        }

        /**
         * Gets the value of the first member.
         *
         * @return The first member's value
         */
        public @Dimensionless int getFirst(Utilities.@Dimensionless IntPair this) {
            return first;
        }

        /**
         * Sets the first member's value.
         *
         * @param first The value for the first member
         */
        public void setFirst(Utilities.@Dimensionless IntPair this, @Dimensionless int first) {
            this.first = first;
        }

        /**
         * Gets the value of the second member.
         *
         * @return The second member's value
         */
        public @Dimensionless int getSecond(Utilities.@Dimensionless IntPair this) {
            return second;
        }

        /**
         * Sets the second member's value.
         *
         * @param second The value for the second member
         */
        public void setSecond(Utilities.@Dimensionless IntPair this, @Dimensionless int second) {
            this.second = second;
        }

        /**
         * Swaps both members. First becomes second, second becomes first.
         */
        public void swap(Utilities.@Dimensionless IntPair this) {
            final @Dimensionless int temp = first;
            first = second;
            second = temp;
        }

        @Override
        public @Dimensionless boolean equals(Utilities.@Dimensionless IntPair this, @Dimensionless Object o) {
            if (this == o) {
                return true;
            }
            if (!(o instanceof IntPair)) {
                return false;
            }
            final @Dimensionless IntPair intPair = (@Dimensionless IntPair) o;
            if (first != intPair.getFirst()) {
                return false;
            }
            if (second != intPair.getSecond()) {
                return false;
            }
            return true;
        }

        @Override
        public @Dimensionless int hashCode(Utilities.@Dimensionless IntPair this) {
            @Dimensionless
            int result = first;
            result = ((@Dimensionless int) (31)) * result + second;
            return result;
        }
    }
}
