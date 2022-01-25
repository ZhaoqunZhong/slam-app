// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: alexs.mac@gmail.com (Alex Stewart)

// Configuration options for Ceres.
//
// Do not edit this file, it was automatically configured by CMake when
// Ceres was compiled with the relevant configuration for the machine
// on which Ceres was compiled.
//
// Ceres Developers: All options should have the same name as their mapped
//                   CMake options, in the preconfigured version of this file
//                   all options should be enclosed in '@'.

#ifndef CERES_PUBLIC_INTERNAL_CONFIG_H_
#define CERES_PUBLIC_INTERNAL_CONFIG_H_

// If defined, use the LGPL code in Eigen.
#define CERES_USE_EIGEN_SPARSE

// If defined, Ceres was compiled without LAPACK.
#define CERES_NO_LAPACK

// If defined, Ceres was compiled without SuiteSparse.
#define CERES_NO_SUITESPARSE

// If defined, Ceres was compiled without CXSparse.
#define CERES_NO_CXSPARSE

// If defined, Ceres was compiled without Apple's Accelerate framework solvers.
#define CERES_NO_ACCELERATE_SPARSE

#if defined(CERES_NO_SUITESPARSE) &&              \
    defined(CERES_NO_ACCELERATE_SPARSE) &&        \
    defined(CERES_NO_CXSPARSE) &&                 \
    !defined(CERES_USE_EIGEN_SPARSE)  // NOLINT
// If defined Ceres was compiled without any sparse linear algebra support.
#define CERES_NO_SPARSE
#endif

// If defined, Ceres was compiled without Schur specializations.
// #define CERES_RESTRICT_SCHUR_SPECIALIZATION

// If defined, Ceres was compiled to use Eigen instead of hardcoded BLAS
// routines.
// #define CERES_NO_CUSTOM_BLAS

// If defined, Ceres was compiled without multithreading support.
// #define CERES_NO_THREADS
// If defined Ceres was compiled with OpenMP multithreading.
// #define CERES_USE_OPENMP
// If defined Ceres was compiled with modern C++ multithreading.
#define CERES_USE_CXX_THREADS

// If defined, Ceres was built as a shared library.
#define CERES_USING_SHARED_LIBRARY

// If defined, Ceres was compiled with a version MSVC >= 2005 which
// deprecated the standard POSIX names for bessel functions, replacing them
// with underscore prefixed versions (e.g. j0() -> _j0()).
// #define CERES_MSVC_USE_UNDERSCORE_PREFIXED_BESSEL_FUNCTIONS

#endif  // CERES_PUBLIC_INTERNAL_CONFIG_H_
