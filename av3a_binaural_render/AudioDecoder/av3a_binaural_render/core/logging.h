/* Copyright 2021 Beijing Zitiao Network Technology Co.,
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef LOGGING_H
#define LOGGING_H

#include <iostream>


namespace avs3renderer {

inline void LogE(const std::string& fmt) {
    std::cerr << fmt << std::endl;
}

}  // namespace avs3renderer
#endif  // LOGGING_H
