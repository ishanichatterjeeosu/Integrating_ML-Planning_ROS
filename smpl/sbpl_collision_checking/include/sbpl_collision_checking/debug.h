////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
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
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef sbpl_collision_debug_h
#define sbpl_collision_debug_h

// COMPILE-TIME ASSERT = no assert
// RUNTIME UNRECOVERABLE ASSERT = assert
// RUNTIME RECOVERABLE ASSERT = exception

#define RANGE_ASSERT_COMPILE_TIME 0             // no assertion
#define RANGE_ASSERT_RUNTIME_UNRECOVERABLE 1    // assert macro
#define RANGE_ASSERT_RUNTIME_RECOVERABLE 2      // std::out_of_range exception
#define RANGE_ASSERT_METHOD RANGE_ASSERT_COMPILE_TIME

#if RANGE_ASSERT_METHOD == RANGE_ASSERT_RUNTIME_RECOVERABLE
#define ASSERT_RANGE(cond) \
{\
    if (!(cond)) {\
        throw std::out_of_range(#cond);\
    }\
}
#elif RANGE_ASSERT_METHOD == RANGE_ASSERT_RUNTIME_UNRECOVERABLE
#define ASSERT_RANGE(cond) \
{\
    assert(cond);\
}
#else
#define ASSERT_RANGE(cond)
#endif

#define ASSERT_VECTOR_RANGE(vector, index) \
ASSERT_RANGE(index >= 0 && index < vector.size());

#endif
