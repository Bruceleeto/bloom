cmake_minimum_required(VERSION 3.10)
project(lightrec LANGUAGES C VERSION 0.9)

# emitter.c and regcache.c are gone: they WERE the GNU Lightning backend, and
# what replaces them is fgl, whose sources live in ../../src/fgl and are added
# below. constprop.c and optimizer.c stay -- the optimiser's proofs are what
# let fgl emit two instructions for a load instead of a call, so they are the
# reason this is worth doing at all.
list(APPEND LIGHTREC_SOURCES
	blockcache.c
	constprop.c
	interpreter.c
	lightrec.c
	memmanager.c
	optimizer.c
)
list(APPEND LIGHTREC_HEADERS
	blockcache.h
	constprop.h
	debug.h
	disassembler.h
	interpreter.h
	lightrec-private.h
	lightrec.h
	memmanager.h
	optimizer.h
	recompiler.h
)

add_library(lightrec ${LIGHTREC_SOURCES} ${LIGHTREC_HEADERS})
set_target_properties(lightrec PROPERTIES
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_VERSION_MAJOR}
	PUBLIC_HEADER lightrec.h
	C_STANDARD 11
	C_STANDARD_REQUIRED ON
	C_EXTENSIONS OFF
)

set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries")
if (NOT BUILD_SHARED_LIBS)
	target_compile_definitions(lightrec PRIVATE LIGHTREC_STATIC)
endif (NOT BUILD_SHARED_LIBS)

if (NOT LOG_LEVEL)
	set(LOG_LEVEL Info CACHE STRING "Log level" FORCE)
	set_property(CACHE LOG_LEVEL PROPERTY STRINGS NoLog Error Warning Info Debug)
endif()

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
		"Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel."
		FORCE)
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS None Debug Release RelWithDebInfo MinSizeRel)
endif()

string(TOUPPER ${LOG_LEVEL} LIGHTREC_LOG_LEVEL)
target_compile_definitions(lightrec PRIVATE LOG_LEVEL=${LIGHTREC_LOG_LEVEL}_L)

if (CMAKE_COMPILER_IS_GNUCC)
	target_compile_options(lightrec PRIVATE -fvisibility=hidden)
endif()

set(HAS_DEFAULT_ELM ${CMAKE_COMPILER_IS_GNUCC})

option(ENABLE_FIRST_PASS "Run the interpreter as first-pass optimization" ON)

option(ENABLE_THREADED_COMPILER "Enable threaded compiler" ON)
if (ENABLE_THREADED_COMPILER)
	target_sources(lightrec PRIVATE recompiler.c reaper.c)

	if (NOT ENABLE_FIRST_PASS)
		message(SEND_ERROR "Threaded compiler requires first-pass optimization")
	endif (NOT ENABLE_FIRST_PASS)
endif (ENABLE_THREADED_COMPILER)

option(OPT_REMOVE_DIV_BY_ZERO_SEQ "(optimization) Remove div-by-zero check sequence" ON)
option(OPT_REPLACE_MEMSET "(optimization) Detect and replace memset with host variant" ON)
option(OPT_DETECT_IMPOSSIBLE_BRANCHES "(optimization) Detect impossible branches" ON)
option(OPT_HANDLE_LOAD_DELAYS "(optimization) Detect load delays" ON)
option(OPT_TRANSFORM_OPS "(optimization) Transform opcodes" ON)
option(OPT_LOCAL_BRANCHES "(optimization) Detect local branches" ON)
option(OPT_SWITCH_DELAY_SLOTS "(optimization) Switch delay slots" ON)
option(OPT_FLAG_IO "(optimization) Flag I/O opcodes when the target can be detected" ON)
option(OPT_FLAG_MULT_DIV "(optimization) Flag MULT/DIV that only use one of HI/LO" ON)
option(OPT_EARLY_UNLOAD "(optimization) Unload registers early" ON)
option(OPT_PRELOAD_PC "(optimization) Preload PC value into register" ON)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "SH4|sh4")
	option(OPT_SH4_USE_GBR "(SH4 optimization) Use GBR register for the state pointer" OFF)
endif()

target_include_directories(lightrec PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
	target_compile_options(lightrec PRIVATE -Wall)
endif()
if (CMAKE_C_COMPILER_ID STREQUAL "Clang")
	target_compile_options(lightrec PRIVATE -Wno-initializer-overrides)
endif()

if (ENABLE_THREADED_COMPILER)
	include(FindThreads)

	if (NOT CMAKE_USE_PTHREADS_INIT)
		message(SEND_ERROR "Could not find compatible threads library")
	endif()

	target_link_libraries(lightrec PUBLIC Threads::Threads)
endif (ENABLE_THREADED_COMPILER)

option(ENABLE_CODE_BUFFER "Enable external code buffer" ON)
if (ENABLE_CODE_BUFFER)
	target_sources(lightrec PRIVATE tlsf/tlsf.c)
	target_include_directories(lightrec PRIVATE tlsf)
endif (ENABLE_CODE_BUFFER)

# fgl -- the SH-4 code generator, in place of GNU Lightning.
#
# The library that used to be found and linked here is not merely optional now,
# it is absent: nothing in this target refers to it. What took its place is a
# handful of C files and two hand-written assembly ones, compiled straight into
# lightrec rather than linked as a library, because they and lightrec.c share a
# private header (lightrec-private.h) and a state layout that only holds
# because src/fgl/fgl_lightrec.c asserts it at compile time.
#
# The .S files need no special handling: ASM is enabled for the whole project
# and gcc preprocesses an uppercase .S, which is what lets them #include
# fgl_state.h and write the same displacements the emitter bakes in.
set(FGL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src/fgl)
target_sources(lightrec PRIVATE
	${FGL_DIR}/decode.c
	${FGL_DIR}/front.c
	${FGL_DIR}/alloc.c
	${FGL_DIR}/emit.c
	${FGL_DIR}/fgl_lightrec.c
	${FGL_DIR}/dispatch.S
	${FGL_DIR}/shim.S
)
target_include_directories(lightrec PRIVATE
	${FGL_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

if (LOG_LEVEL STREQUAL Debug)
	set(ENABLE_DISASSEMBLER ON)
	target_sources(lightrec PRIVATE disassembler.c)
endif()

configure_file(lightrec-config.h.cmakein lightrec-config.h @ONLY)

include(GNUInstallDirs)
install(TARGETS lightrec
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
