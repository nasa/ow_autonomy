#**************************************************************************#
#* Copyright © 2014-2019, United States Government, as represented by the *#
#* Administrator of the National Aeronautics and Space Administration.    *#
#* All rights reserved.                                                   *#
#*                                                                        *#
#* The PLEXIL cFE application is licensed under the Apache License,       *#
#* Version 2.0 (the "License"); you may not use this file except in       *#
#* compliance with the License. You may obtain a copy of the License at   *#
#*                                                                        *#
#*  http://www.apache.org/licenses/LICENSE-2.0                            *#
#*                                                                        *#
#* Unless required by applicable law or agreed to in writing, software    *#
#* distributed under the License is distributed on an "AS IS" BASIS,      *#
#* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        *#
#* implied. See the License for the specific language governing           *#
#* permissions and limitations under the License.                         *#
#**************************************************************************#

# Excerpted from aos/cfe/cmake/arch_build.cmake
#

##################################################################
#
# FUNCTION: add_plexil_plans
#
# Routine to add Plexil plans to an app
#
function(add_plexil_plans APP_NAME PLAN_SRC_FILES)
  set(PLAN_LIST)
  foreach(PLAN ${PLAN_SRC_FILES} ${ARGN})
    # Get name without extension (NAME_WE) and append to list of tables
    get_filename_component(PLANWE ${PLAN} NAME_WE)
    get_filename_component(PLANEX ${PLAN} EXT)
    get_filename_component(PLANRD ${PLAN} DIRECTORY)

    list(APPEND PLAN_LIST "${PLANWE}.plx")

    add_custom_command(
      OUTPUT ${PLANWE}.plx
      COMMAND ${PLEXIL_COMPILER} -o ${PLANWE}.plx ${CMAKE_CURRENT_SOURCE_DIR}/${PLANWE}${PLANEX}
      DEPENDS ${PLAN}
    )

  endforeach(PLAN ${PLAN_SRC_FILES} ${ARGN})

  # Make a custom target that depends on all the plans  
  add_custom_target(${APP_NAME}_plans ALL DEPENDS ${PLAN_LIST})

  foreach(TGT ${APP_INSTALL_LIST})
    foreach(PLAN_FILE ${PLAN_LIST})
      install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/${PLAN_FILE}
        DESTINATION ${TGT}/data/plx/plans
        )
    endforeach(PLAN_FILE ${PLAN_LIST})
  endforeach(TGT ${APP_INSTALL_LIST})

endfunction(add_plexil_plans)
