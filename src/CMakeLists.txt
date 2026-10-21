if(NOT CMAKE_BUILD_EARLY_EXPANSION AND
   (NOT CONFIG_MODEL_IN_FLASH OR NOT CONFIG_SR_VADN_VADNET1_MEDIUM))
    message(FATAL_ERROR
        "AFE requires MODEL_IN_FLASH and SR_VADN_VADNET1_MEDIUM; select vadnet1 medium in ESP Speech Recognition")
endif()

if(NOT CMAKE_BUILD_EARLY_EXPANSION AND NOT CONFIG_SPIRAM_XIP_FROM_PSRAM)
    message(FATAL_ERROR
        "Custom HID model updates require SPIRAM_XIP_FROM_PSRAM so USB audio can run during flash erase/write")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/generated_sources.cmake")

if(${IDF_VERSION_MAJOR} LESS 6)
    set(priv_requires esp_timer driver hal usb)
else()
    set(priv_requires
        esp_timer
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_i2s
        esp_driver_rmt
        esp_driver_uart
        esp_hal_usb
        esp_hw_support
        vfs
    )
endif()

set(app_sources
    "main.c"
    "cpu_usage.c"
    "audio_backend.c"
    "audio_diagnostics.c"
    "audio_console_gate.c"
    "keyword_actions.c"
    "custom/custom_protocol.c"
    "custom/custom_state.c"
    "custom/custom_storage.c"
    "custom/custom_control.c"
    "custom/custom_hid.c"
    "audio_processing/array_dsp.cpp"
    "audio_processing/audio_pipeline.cpp"
    "audio_processing/nscore/enc_core.c"
    "audio_processing/nscore/ns_fft.c"
    "kws/kws_port.cpp"
    "kws/kws_modules.cpp"
    "kws/kws_memory_compat.cpp"
    "kws/kws_frontend.c"
    "kws/kws_stream.cpp"
    "pcmd3180/pcmd3180.c"
    "usb_driver/usb_audio.c"
    "usb_driver/usb_descriptors.c"
    "usb_driver/usb_hid_keyboard.c"
)

if(CONFIG_WS2812_LED_RING_ENABLE)
    list(APPEND app_sources
        "ws2812/ws2812.c"
        "ws2812/led_ring_effects.c"
        "ws2812/led_ring.c"
    )
endif()

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS
        "."
        "pcmd3180"
        "usb_driver"
        "ws2812"
        "audio_processing"
        "kws/include"
        "custom"
    PRIV_REQUIRES ${priv_requires} nvs_flash esp_partition esp_app_format mbedtls espressif__cjson
    WHOLE_ARCHIVE
)

include("${COMPONENT_DIR}/kws/model.cmake")
include("${COMPONENT_DIR}/kws/espdl_budget.cmake")
target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=_write_r")
target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=_ZN2dl4tool16memory_addr_typeEPv")

# The managed TinyUSB component must see this component's tusb_config.h while
# TinyUSB itself is being compiled.
idf_build_get_property(build_components BUILD_COMPONENTS)
if(tinyusb IN_LIST build_components)
    set(tinyusb_component tinyusb)
else()
    set(tinyusb_component espressif__tinyusb)
endif()
idf_component_get_property(tinyusb_lib ${tinyusb_component} COMPONENT_LIB)
target_include_directories(${tinyusb_lib} PUBLIC "${COMPONENT_DIR}/usb_driver")

# TinyUSB 0.19 schedules the next isochronous IN transfer before notifying the
# application. Capture2 can have a full DSP FIFO at that point, yet TinyUSB has
# already committed a zero-length packet from its still-empty endpoint FIFO.
# Build a local copy that refills first only when the endpoint FIFO is empty.
# With queued data, arm the next transfer immediately and refill afterwards so
# the DWC2 isochronous endpoint has as much time as possible before its slot.
# Keep the managed component pristine and fail loudly if its source layout
# changes in a future dependency update.
get_target_property(tinyusb_sources ${tinyusb_lib} SOURCES)
get_target_property(tinyusb_source_dir ${tinyusb_lib} SOURCE_DIR)
set(patched_tinyusb_sources "")
set(tinyusb_audio_patched FALSE)
set(tinyusb_dwc2_patched FALSE)
foreach(tinyusb_source IN LISTS tinyusb_sources)
    if(tinyusb_source MATCHES "(^|/)src/class/audio/audio_device[.]c$")
        if(IS_ABSOLUTE "${tinyusb_source}")
            set(tinyusb_audio_source "${tinyusb_source}")
        else()
            set(tinyusb_audio_source "${tinyusb_source_dir}/${tinyusb_source}")
        endif()
        get_filename_component(tinyusb_audio_dir "${tinyusb_audio_source}" DIRECTORY)
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${tinyusb_audio_source}")
        target_include_directories(${tinyusb_lib} PRIVATE "${tinyusb_audio_dir}")
        file(READ "${tinyusb_audio_source}" tinyusb_audio_text)
        set(old_audio_include [=[#include "audio_device.h"]=])
        set(new_audio_include [=[#include "audio_device.h"
extern void usb_audio_capture_xfer_result_isr(uint8_t func_id, uint8_t result, uint16_t bytes);
extern void usb_audio_capture_xfer_scheduled_isr(uint8_t func_id, uint16_t bytes);]=])
        set(old_tx_begin [=[  // Send everything in ISO EP FIFO
  uint16_t n_bytes_tx;]=])
        set(new_tx_begin [=[  // Refill first only when an empty FIFO would schedule a ZLP.
  // Otherwise arm DWC2 before running the application callback.
  bool const refill_first = tu_fifo_count(&audio->ep_in_ff) == 0;
  if (refill_first) {
    TU_VERIFY(tud_audio_tx_done_isr(rhport, n_bytes_sent, idx_audio_fct, audio->ep_in, audio->ep_in_alt));
  }

  // Send everything in ISO EP FIFO
  uint16_t n_bytes_tx;]=])
        set(old_tx_end [=[  // Call a weak callback here - a possibility for user to get informed former TX was completed and data gets now loaded into EP in buffer
  TU_VERIFY(tud_audio_tx_done_isr(rhport, n_bytes_sent, idx_audio_fct, audio->ep_in, audio->ep_in_alt));

  return true;]=])
        set(new_tx_end [=[  usb_audio_capture_xfer_scheduled_isr(idx_audio_fct, n_bytes_tx);
  if (!refill_first) {
    TU_VERIFY(tud_audio_tx_done_isr(rhport, n_bytes_sent, idx_audio_fct, audio->ep_in, audio->ep_in_alt));
  }
  return true;]=])
        set(old_xfer_done [=[      audiod_tx_xfer_isr(rhport, audio, (uint16_t) xferred_bytes);]=])
        set(new_xfer_done [=[      usb_audio_capture_xfer_result_isr(func_id, (uint8_t) result, (uint16_t) xferred_bytes);
      audiod_tx_xfer_isr(rhport, audio, (uint16_t) xferred_bytes);]=])
        set(old_feedback_start [=[            // Schedule first feedback transmit
            audiod_fb_send(audio);]=])
        set(new_feedback_start [=[            // The application supplies the initial feedback in set_itf_cb.]=])
        set(old_feedback_after_itf [=[      TU_VERIFY(tud_audio_set_itf_cb(rhport, p_request));

#if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP]=])
        set(new_feedback_after_itf [=[      TU_VERIFY(tud_audio_set_itf_cb(rhport, p_request));

#if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
      // Start feedback only for the OUT interface just opened. A microphone
      // SET_INTERFACE must not overwrite/re-submit an outstanding feedback DMA.
      if (audio->ep_fb != 0 && audio->ep_out_as_intf_num == itf && alt != 0) {
        audiod_fb_send(audio);
      }]=])
        set(old_rx_fifo_write [=[  // Data currently is in linear buffer, copy into EP OUT FIFO
  TU_VERIFY(tu_fifo_write_n(&audio->ep_out_ff, audio->lin_buf_out, n_bytes_received));]=])
        set(new_rx_fifo_write [=[  // A ZLP (including a PTI packet-drop completion) still needs the next
  // receive armed. tu_fifo_write_n(..., 0) returns 0, not a FIFO error.
  if (n_bytes_received != 0) {
    TU_VERIFY(tu_fifo_write_n(&audio->ep_out_ff, audio->lin_buf_out, n_bytes_received));
  }]=])
        string(FIND "${tinyusb_audio_text}" "${old_tx_begin}" tx_begin_pos)
        string(FIND "${tinyusb_audio_text}" "${old_tx_end}" tx_end_pos)
        string(FIND "${tinyusb_audio_text}" "${old_audio_include}" audio_include_pos)
        string(FIND "${tinyusb_audio_text}" "${old_xfer_done}" xfer_done_pos)
        string(FIND "${tinyusb_audio_text}" "${old_feedback_start}" feedback_start_pos)
        string(FIND "${tinyusb_audio_text}" "${old_feedback_after_itf}" feedback_after_itf_pos)
        string(FIND "${tinyusb_audio_text}" "${old_rx_fifo_write}" rx_fifo_write_pos)
        if(tx_begin_pos EQUAL -1 OR tx_end_pos EQUAL -1 OR
           audio_include_pos EQUAL -1 OR xfer_done_pos EQUAL -1 OR
           feedback_start_pos EQUAL -1 OR feedback_after_itf_pos EQUAL -1 OR
           rx_fifo_write_pos EQUAL -1)
            message(FATAL_ERROR "TinyUSB audio callback layout changed; update the USB transfer-order patch")
        endif()
        string(REPLACE "${old_tx_begin}" "${new_tx_begin}" tinyusb_audio_text "${tinyusb_audio_text}")
        string(REPLACE "${old_tx_end}" "${new_tx_end}" tinyusb_audio_text "${tinyusb_audio_text}")
        string(REPLACE "${old_audio_include}" "${new_audio_include}" tinyusb_audio_text "${tinyusb_audio_text}")
        string(REPLACE "${old_xfer_done}" "${new_xfer_done}" tinyusb_audio_text "${tinyusb_audio_text}")
        string(REPLACE "${old_feedback_start}" "${new_feedback_start}" tinyusb_audio_text "${tinyusb_audio_text}")
        string(REPLACE "${old_feedback_after_itf}" "${new_feedback_after_itf}" tinyusb_audio_text "${tinyusb_audio_text}")
        string(REPLACE "${old_rx_fifo_write}" "${new_rx_fifo_write}" tinyusb_audio_text "${tinyusb_audio_text}")
        set(patched_audio_source "${CMAKE_CURRENT_BINARY_DIR}/audio_device_capture2.c")
        write_generated_source("${patched_audio_source}" "${tinyusb_audio_text}")
        list(APPEND patched_tinyusb_sources "${patched_audio_source}")
        set(tinyusb_audio_patched TRUE)
    elseif(tinyusb_source MATCHES "(^|/)src/portable/synopsys/dwc2/dcd_dwc2[.]c$")
        if(IS_ABSOLUTE "${tinyusb_source}")
            set(tinyusb_dwc2_source "${tinyusb_source}")
        else()
            set(tinyusb_dwc2_source "${tinyusb_source_dir}/${tinyusb_source}")
        endif()
        get_filename_component(tinyusb_dwc2_dir "${tinyusb_dwc2_source}" DIRECTORY)
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${tinyusb_dwc2_source}")
        target_include_directories(${tinyusb_lib} PRIVATE "${tinyusb_dwc2_dir}")
        file(READ "${tinyusb_dwc2_source}" tinyusb_dwc2_text)
        # Backport TinyUSB's DMA-only periodic transfer mode. It lets DWC2
        # retain an armed ISO IN packet across a missed microframe instead of
        # exhausting the software retry count and reporting a failed packet.
        set(old_dwc2_controller_start [=[//--------------------------------------------------------------------
// Controller API
//--------------------------------------------------------------------
bool dcd_init]=])
        set(new_dwc2_controller_start [=[static void capture_pti_configure(dwc2_regs_t* dwc2) {
#if CFG_TUD_DWC2_PTI_ENABLE
  if (!dma_device_enabled(dwc2)) return;
  dwc2_dctl_t dctl = {.value = dwc2->dctl};
  dctl.ignore_frame_number = 1;
  dwc2->dctl = dctl.value;
  const dwc2_ghwcfg4_t ghwcfg4 = {.value = dwc2->ghwcfg4};
  if (ghwcfg4.dedicated_fifos) {
    dwc2->dthrctl &= ~(DTHRCTL_NONISOTHREN | DTHRCTL_ISOTHREN | DTHRCTL_RXTHREN);
  }
#else
  (void) dwc2;
#endif
}

//--------------------------------------------------------------------
// Controller API
//--------------------------------------------------------------------
bool dcd_init]=])
        set(old_dwc2_init [=[  dwc2->dcfg = dcfg;

  dcd_disconnect(rhport);]=])
        set(new_dwc2_init [=[  dwc2->dcfg = dcfg;
  capture_pti_configure(dwc2);

  dcd_disconnect(rhport);]=])
        set(old_dwc2_reset [=[  dwc2->dcfg = dcfg.value;

  // 6. Configure maximum packet size for EP0]=])
        set(new_dwc2_reset [=[  dwc2->dcfg = dcfg.value;
  capture_pti_configure(dwc2);

  // 6. Configure maximum packet size for EP0]=])
        set(old_dwc2_iso_mask [=[  dwc2->gintmsk |= GINTMSK_OEPINT | GINTMSK_IEPINT | GINTMSK_IISOIXFRM;]=])
        set(new_dwc2_iso_mask [=[  dwc2->gintmsk &= ~GINTMSK_IISOIXFRM;
  dwc2->gintmsk |= GINTMSK_OEPINT | GINTMSK_IEPINT;
  if (!(CFG_TUD_DWC2_PTI_ENABLE && dma_device_enabled(dwc2))) {
    dwc2->gintmsk |= GINTMSK_IISOIXFRM;
  }]=])
        # PTI also changes ISO OUT completion semantics: a dropped packet may
        # raise XFRC without valid payload. Complete it as zero bytes so the
        # audio driver re-arms OUT without reading stale data or underflowing
        # total_len when the residual transfer size exceeds the request.
        set(old_dwc2_out_complete [=[    if (!doepint_bm.status_phase_rx && !doepint_bm.setup_packet_rx) {
      if ((epnum == 0) && _dcd_data.ep0_pending[TUSB_DIR_OUT]) {]=])
        set(new_dwc2_out_complete [=[    if (!doepint_bm.status_phase_rx && !doepint_bm.setup_packet_rx) {
#if CFG_TUD_DWC2_PTI_ENABLE
      if (doepint_bm.iso_packet_drop && dma_device_enabled(dwc2)) {
        const dwc2_depctl_t depctl = {.value = dwc2->epout[epnum].doepctl};
        if (depctl.type == DEPCTL_EPTYPE_ISOCHRONOUS) {
          dcd_event_xfer_complete(rhport, epnum, 0, XFER_RESULT_FAILED, true);
          return;
        }
      }
#endif
      if ((epnum == 0) && _dcd_data.ep0_pending[TUSB_DIR_OUT]) {]=])
        string(FIND "${tinyusb_dwc2_text}" "${old_dwc2_controller_start}" dwc2_controller_pos)
        string(FIND "${tinyusb_dwc2_text}" "${old_dwc2_init}" dwc2_init_pos)
        string(FIND "${tinyusb_dwc2_text}" "${old_dwc2_reset}" dwc2_reset_pos)
        string(FIND "${tinyusb_dwc2_text}" "${old_dwc2_iso_mask}" dwc2_iso_mask_pos)
        string(FIND "${tinyusb_dwc2_text}" "${old_dwc2_out_complete}" dwc2_out_complete_pos)
        if(dwc2_controller_pos EQUAL -1 OR dwc2_init_pos EQUAL -1 OR
           dwc2_reset_pos EQUAL -1 OR dwc2_iso_mask_pos EQUAL -1 OR
           dwc2_out_complete_pos EQUAL -1)
            message(FATAL_ERROR "TinyUSB DWC2 layout changed; update the PTI patch")
        endif()
        string(REPLACE "${old_dwc2_controller_start}" "${new_dwc2_controller_start}" tinyusb_dwc2_text "${tinyusb_dwc2_text}")
        string(REPLACE "${old_dwc2_init}" "${new_dwc2_init}" tinyusb_dwc2_text "${tinyusb_dwc2_text}")
        string(REPLACE "${old_dwc2_reset}" "${new_dwc2_reset}" tinyusb_dwc2_text "${tinyusb_dwc2_text}")
        string(REPLACE "${old_dwc2_iso_mask}" "${new_dwc2_iso_mask}" tinyusb_dwc2_text "${tinyusb_dwc2_text}")
        string(REPLACE "${old_dwc2_out_complete}" "${new_dwc2_out_complete}" tinyusb_dwc2_text "${tinyusb_dwc2_text}")
        set(patched_dwc2_source "${CMAKE_CURRENT_BINARY_DIR}/dcd_dwc2_capture_pti.c")
        write_generated_source("${patched_dwc2_source}" "${tinyusb_dwc2_text}")
        list(APPEND patched_tinyusb_sources "${patched_dwc2_source}")
        set(tinyusb_dwc2_patched TRUE)
    else()
        list(APPEND patched_tinyusb_sources "${tinyusb_source}")
    endif()
endforeach()
if(NOT tinyusb_audio_patched OR NOT tinyusb_dwc2_patched)
    message(FATAL_ERROR "TinyUSB audio or DWC2 source was not found for the local patches")
endif()
set_property(TARGET ${tinyusb_lib} PROPERTY SOURCES "${patched_tinyusb_sources}")

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

include("${COMPONENT_DIR}/audio_processing/gain_calibration.cmake")
idf_build_get_property(array_gain_python PYTHON)
generate_array_gain_header("${array_gain_python}" "${CMAKE_CURRENT_BINARY_DIR}")
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
