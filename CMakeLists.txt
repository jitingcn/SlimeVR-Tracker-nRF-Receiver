#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

# Configure build metadata from git (best-effort)
find_package(Git)

if(GIT_EXECUTABLE)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} branch --show-current
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    )
  # Prefer a https URL for display (falls back to 'unknown')
  execute_process(
    COMMAND ${GIT_EXECUTABLE} config --get remote.origin.url
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_REPO_URL_SSH
    )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} config --get remote.origin.https.url
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_REPO_URL_HTTPS
    )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} log -1 --pretty=%an
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_AUTHOR
    )
endif()

if(GIT_BRANCH STREQUAL "")
  set(GIT_BRANCH unknown)
  message(WARNING "Unknown Git Branch")
endif()
# Choose URL: origin.https.url > origin.url
if(NOT GIT_REPO_URL_HTTPS STREQUAL "")
  set(GIT_REPO_URL ${GIT_REPO_URL_HTTPS})
else()
  set(GIT_REPO_URL ${GIT_REPO_URL_SSH})
endif()

# Normalize GitHub SSH URLs to HTTPS for display in firmware info
if(GIT_REPO_URL MATCHES "^git@github\\.com:(.+)$")
  set(GIT_REPO_URL "https://github.com/${CMAKE_MATCH_1}")
elseif(GIT_REPO_URL MATCHES "^ssh://git@github\\.com/(.+)$")
  set(GIT_REPO_URL "https://github.com/${CMAKE_MATCH_1}")
endif()
string(REGEX REPLACE "\\.git$" "" GIT_REPO_URL "${GIT_REPO_URL}")

if(GIT_REPO_URL STREQUAL "")
  set(GIT_REPO_URL unknown)
  message(WARNING "Unknown Git Repo URL")
endif()
if(GIT_AUTHOR STREQUAL "")
  set(GIT_AUTHOR unknown)
  message(WARNING "Unknown Git Author")
endif()

string(STRIP ${GIT_BRANCH} GIT_BRANCH)
string(STRIP ${GIT_REPO_URL} GIT_REPO_URL)
string(STRIP ${GIT_AUTHOR} GIT_AUTHOR)

# Pass metadata to C compilation units
add_compile_definitions(
  FW_GIT_BRANCH="${GIT_BRANCH}"
  FW_GIT_REPO_URL="${GIT_REPO_URL}"
  FW_GIT_AUTHOR="${GIT_AUTHOR}"
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

FILE(GLOB_RECURSE app_sources src/*.c)
# Data collection: choose HID or CDC implementation, or exclude both
if(CONFIG_DATA_COLLECT)
  if(CONFIG_DATA_COLLECT_HID)
    # HID mode: exclude CDC implementation
    list(FILTER app_sources EXCLUDE REGEX ".*data_collect\\.c$")
  else()
    # CDC mode: exclude HID implementation
    list(FILTER app_sources EXCLUDE REGEX ".*data_collect_hid\\.c$")
  endif()
else()
  # No data collection: exclude both
  list(FILTER app_sources EXCLUDE REGEX ".*data_collect.*\\.c$")
endif()
# NORDIC SDK APP START
target_sources(app PRIVATE ${app_sources})
# NORDIC SDK APP END

target_include_directories(app PRIVATE src)

# Force recompile for updated build timestamp
add_custom_target(touch_util_h ALL
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_SOURCE_DIR}/src/build_defines.h
)
