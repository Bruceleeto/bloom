cmake_minimum_required(VERSION 3.10)
project(lightrec LANGUAGES C VERSION 0.9)

# disassembler.c is no longer Debug-only: it holds `mips_regs[]` and
# `lightrec_reg_name()`, which the register cache names whatever the log level
# is.  Everything that IS the disassembler still compiles out behind
# ENABLE_DISASSEMBLER, so the cost of having it here is a 34-entry name table.
#
# ONE BACKEND PER IMAGE.  WITH_FGL replaces GNU Lightning rather than joining
# it: `emitter.c` and `regcache.c` ARE the Lightning backend, and the rest of
# lightrec.c's use of it is behind LIGHTREC_NO_LIGHTNING, so an fgl build does
# not compile, link, or carry a byte of either.  Both can be built at once --
# `lightrec_select_backend` picks by name at run time and that is how the two
# were compared -- but a Dreamcast image has no reason to carry the loser.
option(WITH_FGL "Build the fgl SH-4 code generator instead of GNU Lightning" ON)

list(APPEND LIGHTREC_SOURCES
	blockcache.c
	constprop.c
	disassembler.c
	interpreter.c
	lightrec.c
	memmanager.c
	optimizer.c
)
if (NOT WITH_FGL)
	list(APPEND LIGHTREC_SOURCES
		emitter.c
		regcache.c
	)
endif()
list(APPEND LIGHTREC_HEADERS
	lightrec-backend.h
	blockcache.h
	constprop.h
	debug.h
	disassembler.h
	emitter.h
	interpreter.h
	lightrec-private.h
	lightrec.h
	memmanager.h
	optimizer.h
	recompiler.h
	regcache.h
)

# THE DREAMCAST HAS NO DYNAMIC LINKING.  BUILD_SHARED_LIBS is ON in this
# tree's cache (it is set below, and a cached value is already in scope on
# every configure after the first), so a bare add_library asks for SHARED,
# CMake warns, and builds STATIC anyway.  Asking for what it is going to do
# changes nothing about the output and drops the warning.  BUILD_SHARED_LIBS
# itself is left alone -- it still decides LIGHTREC_STATIC below.
get_property(LIGHTREC_CAN_SHARE GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS)
if (BUILD_SHARED_LIBS AND NOT LIGHTREC_CAN_SHARE)
	add_library(lightrec STATIC ${LIGHTREC_SOURCES} ${LIGHTREC_HEADERS})
else()
	add_library(lightrec ${LIGHTREC_SOURCES} ${LIGHTREC_HEADERS})
endif()
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

option(ENABLE_THREADED_COMPILER "Enable threaded compiler" OFF)
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
if (OPT_LOCAL_BRANCHES)
	option(OPT_DETECT_IDLE "(optimization) Detect idle loops" ON)
else()
	set(OPT_DETECT_IDLE OFF)
endif()

if (CMAKE_SYSTEM_PROCESSOR MATCHES "SH4|sh4")
	option(OPT_SH4_USE_GBR "(SH4 optimization) Use GBR register for the state pointer" OFF)
endif()

# ${CMAKE_BINARY_DIR} is where bloom's configure_file drops bloom-config.h,
# which prof.h reads for the BLOOM_PROF switch.
target_include_directories(lightrec PRIVATE
	${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_BINARY_DIR}
)

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

# No Lightning to find when fgl replaced it: nothing left in this library
# includes lightning.h, so requiring it would be asking for a dependency that
# the build has deliberately dropped.
if (NOT WITH_FGL)
	find_library(LIBLIGHTNING lightning REQUIRED)
	find_path(LIBLIGHTNING_INCLUDE_DIR lightning.h REQUIRED)

	target_include_directories(lightrec PUBLIC ${LIBLIGHTNING_INCLUDE_DIR})
	target_link_libraries(lightrec PUBLIC ${LIBLIGHTNING})
endif (NOT WITH_FGL)

if (LOG_LEVEL STREQUAL Debug)
	set(ENABLE_DISASSEMBLER ON)
endif()

# ---------------------------------------------------------------------------
# fgl -- a second code generator behind the same seam.
#
# lightrec-backend.h turned the code generator into nine function pointers, so
# a build can hold BOTH: GNU Lightning above and fgl here, chosen by name at
# run time rather than by #ifdef.  Nothing on that path is hot -- `compile`
# runs once per block and `execute` about twice a frame -- so the indirect call
# costs nothing measurable, and having both in one image is what makes an A/B
# on hardware a menu entry instead of a rebuild.
#
# fgl's sources are compiled straight into lightrec rather than linked as a
# library because they and lightrec.c share a private header and a state
# layout that only holds because src/fgl/fgl_lightrec.c asserts it at compile
# time.
#
# The .S files need no special handling: ASM is enabled for the whole project
# and gcc preprocesses an uppercase .S, which is what lets them #include
# fgl_state.h and write the same displacements the emitter bakes in.
# ---------------------------------------------------------------------------
if (WITH_FGL)
	set(FGL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src/fgl)
	target_sources(lightrec PRIVATE
		${FGL_DIR}/decode.c
		${FGL_DIR}/front.c
		${FGL_DIR}/alloc.c
		${FGL_DIR}/emit.c
		${FGL_DIR}/fgl_lightrec.c
		${FGL_DIR}/lightrec_backend.c
		${FGL_DIR}/fgl_link.c
		${FGL_DIR}/fgl_targets.c
		${FGL_DIR}/gte_fpu.c
	)
	if (CMAKE_SYSTEM_PROCESSOR MATCHES "SH4|sh4" OR KOS_BASE)
		# The dispatcher, the C shim and the RTPS/RTPT leaf: hand-written
		# SH-4, and the reason there is nothing to generate at run time.
		target_sources(lightrec PRIVATE
			${FGL_DIR}/dispatch.S
			${FGL_DIR}/shim.S
			${FGL_DIR}/gte_rtp.S
		)
	endif()
	target_include_directories(lightrec PRIVATE
		${FGL_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/../../src
		${CMAKE_CURRENT_SOURCE_DIR}/../pcsx_rearmed/libpcsxcore
		${CMAKE_CURRENT_SOURCE_DIR}/../pcsx_rearmed/include
		${CMAKE_CURRENT_SOURCE_DIR}/../pcsx_rearmed
	)
	# PUBLIC, and it has to be: LIGHTREC_BACKEND_WORDS sizes a member in the
	# middle of `struct lightrec_state`, so a translation unit that sees the
	# header without the define lays the struct out differently and reads
	# `current_cycle` out of the backend's block.  That is not a link error,
	# it is a guest whose clock never advances.
	target_compile_definitions(lightrec PUBLIC
		LIGHTREC_WITH_FGL
		LIGHTREC_NO_LIGHTNING
		LIGHTREC_BACKEND_WORDS=90
		FGL
	)
endif (WITH_FGL)

configure_file(lightrec-config.h.cmakein lightrec-config.h @ONLY)

include(GNUInstallDirs)
install(TARGETS lightrec
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
