/*
 * Copyright (c) 2016, CATIE, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef CATIE_6TRON_LIBTEMPLATE_H_
#define CATIE_6TRON_LIBTEMPLATE_H_

namespace 6tron
{

class LibTemplate
{
public:
    LibTemplate();

private:
    DigitalOut &led_;

};

}

#endif // CATIE_6TRON_LIBTEMPLATE_H_