# Shared desktop icon pipeline. The SVG remains the sole tracked artwork;
# platform-specific raster formats are generated into the build tree.

set(KPT_ICON_SVG "${CMAKE_CURRENT_LIST_DIR}/../packaging/icons/kpt-workbench.svg")

function(kpt_generate_linux_icons out_icons)
  find_program(KPT_RSVG_CONVERT_EXECUTABLE NAMES rsvg-convert REQUIRED)
  set(outputs)
  foreach(size IN ITEMS 16 32 48 64 128 256)
    set(output "${CMAKE_CURRENT_BINARY_DIR}/icons/${size}x${size}/apps/kpt-workbench.png")
    add_custom_command(
      OUTPUT "${output}"
      COMMAND "${CMAKE_COMMAND}" -E make_directory
        "${CMAKE_CURRENT_BINARY_DIR}/icons/${size}x${size}/apps"
      COMMAND "${KPT_RSVG_CONVERT_EXECUTABLE}" --width "${size}" --height "${size}"
        --output "${output}" "${KPT_ICON_SVG}"
      DEPENDS "${KPT_ICON_SVG}"
      VERBATIM)
    list(APPEND outputs "${output}")
  endforeach()
  add_custom_target(kpt_linux_icons ALL DEPENDS ${outputs})
  set(${out_icons} "${outputs}" PARENT_SCOPE)
endfunction()

function(kpt_attach_windows_icon target)
  find_program(KPT_ICON_MAGICK_EXECUTABLE NAMES magick REQUIRED)
  set(icon_directory "${CMAKE_CURRENT_BINARY_DIR}/generated/icons")
  set(icon "${icon_directory}/kpt-workbench.ico")
  add_custom_command(
    OUTPUT "${icon}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${icon_directory}"
    COMMAND "${KPT_ICON_MAGICK_EXECUTABLE}" -background none "${KPT_ICON_SVG}"
      -define icon:auto-resize=16,20,24,32,40,48,64,256 "${icon}"
    DEPENDS "${KPT_ICON_SVG}"
    VERBATIM)
  add_custom_target(kpt_windows_icon DEPENDS "${icon}")
  set(KPT_WINDOWS_ICON "${icon}")
  configure_file("${CMAKE_CURRENT_FUNCTION_LIST_DIR}/windows-kpt-icon.rc.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/kpt_gui_icon.rc" @ONLY)
  target_sources("${target}" PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/generated/kpt_gui_icon.rc")
  add_dependencies("${target}" kpt_windows_icon)
endfunction()
