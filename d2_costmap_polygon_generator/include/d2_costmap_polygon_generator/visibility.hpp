// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef D2__COSTMAP_POLYGON_GENERATOR__VISIBILITY_HPP_
#define D2__COSTMAP_POLYGON_GENERATOR__VISIBILITY_HPP_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borroexampled (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define D2__COSTMAP_POLYGON_GENERATOR_EXPORT __attribute__((dllexport))
#define D2__COSTMAP_POLYGON_GENERATOR_IMPORT __attribute__((dllimport))
#else
#define D2__COSTMAP_POLYGON_GENERATOR_EXPORT __declspec(dllexport)
#define D2__COSTMAP_POLYGON_GENERATOR_IMPORT __declspec(dllimport)
#endif

#ifdef D2__COSTMAP_POLYGON_GENERATOR_DLL
#define D2__COSTMAP_POLYGON_GENERATOR_PUBLIC D2__COSTMAP_POLYGON_GENERATOR_EXPORT
#else
#define D2__COSTMAP_POLYGON_GENERATOR_PUBLIC D2__COSTMAP_POLYGON_GENERATOR_IMPORT
#endif

#define D2__COSTMAP_POLYGON_GENERATOR_PUBLIC_TYPE D2__COSTMAP_POLYGON_GENERATOR_PUBLIC

#define D2__COSTMAP_POLYGON_GENERATOR_LOCAL

#else

#define D2__COSTMAP_POLYGON_GENERATOR_EXPORT __attribute__((visibility("default")))
#define D2__COSTMAP_POLYGON_GENERATOR_IMPORT

#if __GNUC__ >= 4
#define D2__COSTMAP_POLYGON_GENERATOR_PUBLIC __attribute__((visibility("default")))
#define D2__COSTMAP_POLYGON_GENERATOR_LOCAL __attribute__((visibility("hidden")))
#else
#define D2__COSTMAP_POLYGON_GENERATOR_PUBLIC
#define D2__COSTMAP_POLYGON_GENERATOR_LOCAL
#endif

#define D2__COSTMAP_POLYGON_GENERATOR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // D2__COSTMAP_POLYGON_GENERATOR__VISIBILITY_HPP_
