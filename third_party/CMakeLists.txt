add_subdirectory(spdlog)
add_subdirectory(toml)
add_subdirectory(catch2)
# popl is consumed as a header-only build dependency. Excluding its directory
# also prevents its unconditional upstream install() rule from leaking the
# development header into KPT runtime packages.
add_subdirectory(popl EXCLUDE_FROM_ALL)
add_subdirectory(eigen)
#add_subdirectory(rapidcsv)
