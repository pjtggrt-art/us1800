/* SPDX-License-Identifier: GPL-2.0-only */
// Copyright (c) 2026 Šerif Rami <ramiserifpersia@gmail.com>
// Revised October 2026 Paul Taggart <pjtggrt@gmail.com>

#ifndef __US1800_H
#define __US1800_H

#include <linux/kfifo.h>
#include <linux/timer.h>
#include <linux/usb.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>

#define DRIVER_NAME "us1800"

#define USB_VID_TASCAM 0x0644
#define USB_PID_TASCAM_US1800 0x8030

#define EP_PLAYBACK_FEEDBACK 0x81
#define EP_AUDIO_OUT 0x02
#define EP_MIDI_IN 0x83
#define EP_MIDI_OUT 0x04
#define EP_AUDIO_IN 0x86

#define RT_H2D_CLASS_EP (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_ENDPOINT)
#define RT_D2H_CLASS_EP (USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_ENDPOINT)
#define RT_H2D_VENDOR_DEV (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE)
#define RT_D2H_VENDOR_DEV (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE)

enum uac_request {
	UAC_SET_CUR = 0x01,
	UAC_GET_CUR = 0x81,
};

enum uac_control_selector {
	UAC_SAMPLING_FREQ_CONTROL = 0x0100,
};

enum tascam_vendor_request {
	VENDOR_REQ_POWER_CONTROL = 0x00,
	VENDOR_REQ_REGISTER_WRITE = 0x41,
	VENDOR_REQ_MODE_CONTROL = 0x49,
	VENDOR_REQ_FIRMWARE_READ = 0x56,
};

enum tascam_mode_value {
	MODE_VAL_HANDSHAKE_READ = 0x0000,
	MODE_VAL_WAKE_UP = 0x000d,
	MODE_VAL_CONFIG = 0x0010,
	MODE_VAL_STREAM_START_US1800 = 0x0032,
	MODE_VAL_STREAM_STOP_US1800 = 0x0036,
	MODE_VAL_DEEP_SLEEP = 0x0044,
};

enum tascam_register {
	REG_ADDR_INIT_0D = 0x0d04,
	REG_ADDR_INIT_0E = 0x0e00,
	REG_ADDR_INIT_0F = 0x0f00,
	REG_ADDR_RATE_44100 = 0x1000,
	REG_ADDR_RATE_48000 = 0x1002,
	REG_ADDR_RATE_88200 = 0x1008,
	REG_ADDR_RATE_96000 = 0x100a,
	REG_ADDR_INIT_11 = 0x110b,
};

#define REG_VAL_ENABLE 0x0101

#define NUM_PLAYBACK_URBS 4
#define PLAYBACK_URB_PACKETS 16
#define NUM_FEEDBACK_URBS 4
#define FEEDBACK_URB_PACKETS 1
#define FEEDBACK_PACKET_SIZE 3
#define NUM_CAPTURE_URBS 8
#define CAPTURE_PACKET_SIZE 32768
#define NUM_MIDI_IN_URBS 4
#define MIDI_IN_BUF_SIZE 64
#define MIDI_IN_FIFO_SIZE (MIDI_IN_BUF_SIZE * NUM_MIDI_IN_URBS)
#define MIDI_OUT_BUF_SIZE 64
#define NUM_MIDI_OUT_URBS 4

#define BYTES_PER_SAMPLE 3
#define PLAYBACK_CHANNELS 4
#define CAPTURE_CHANNELS 16

#define PLAYBACK_FRAME_SIZE (PLAYBACK_CHANNELS * BYTES_PER_SAMPLE)
#define MAX_FRAMES_PER_PACKET 13

#define PLL_FILTER_OLD_WEIGHT 3
#define PLL_FILTER_NEW_WEIGHT 1
#define PLL_FILTER_DIVISOR (PLL_FILTER_OLD_WEIGHT + PLL_FILTER_NEW_WEIGHT)

#define USB_CTRL_TIMEOUT_MS 1000

struct us1800_card {
	struct usb_device *dev;
	struct usb_interface *iface0;
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_rawmidi *rmidi;

	u8 *scratch_buf;

	struct snd_pcm_substream *playback_substream;
	struct snd_pcm_substream *capture_substream;

	struct urb *playback_urbs[NUM_PLAYBACK_URBS];
	size_t playback_urb_alloc_size;
	struct urb *feedback_urbs[NUM_FEEDBACK_URBS];
	size_t feedback_urb_alloc_size;
	struct urb *capture_urbs[NUM_CAPTURE_URBS];
	struct urb *midi_in_urbs[NUM_MIDI_IN_URBS];
	struct urb *midi_out_urbs[NUM_MIDI_OUT_URBS];

	struct usb_anchor playback_anchor;
	struct usb_anchor feedback_anchor;
	struct usb_anchor capture_anchor;
	struct usb_anchor midi_in_anchor;
	struct usb_anchor midi_out_anchor;

	spinlock_t lock;
	atomic_t playback_active;
	atomic_t playback_zero;
	atomic_t capture_active;
	atomic_t active_urbs;
	int current_rate;

	u64 playback_frames_consumed;
	snd_pcm_uframes_t driver_playback_pos;
	u64 last_pb_period_pos;

	u64 capture_frames_processed;
	snd_pcm_uframes_t driver_capture_pos;
	u64 last_cap_period_pos;

	/* --- MIDI State --- */
	struct snd_rawmidi_substream *midi_in_substream;
	struct snd_rawmidi_substream *midi_out_substream;
	atomic_t midi_in_active;
	atomic_t midi_out_active;
	struct kfifo midi_in_fifo;
	spinlock_t midi_in_lock;
	spinlock_t midi_out_lock;
	unsigned long midi_out_urbs_in_flight;
	u8 midi_running_status;
	struct timer_list error_timer;
	struct completion midi_out_drain_completion;

	u32 phase_accum;
	u32 freq_q16;
	bool feedback_synced;
	unsigned int feedback_urb_skip_count;
    char *silence;

	bool rate_changing;

	struct work_struct stop_work;
	struct work_struct stop_pcm_work;
	struct work_struct midi_in_work;
	struct work_struct midi_out_work;
};

void us1800_free_urbs(struct us1800_card *us1800);
int us1800_alloc_urbs(struct us1800_card *us1800);
void us1800_stop_work_handler(struct work_struct *work);
void us1800_zero_trigger(struct us1800_card *us1800, bool start);

/* us1800_midi.c */
/**
 * tascam_midi_in_urb_complete() - Completion handler for MIDI IN URBs
 * @urb: The completed URB.
 *
 * This function runs in interrupt context. It places the raw data from the
 * USB endpoint into a kfifo and schedules a work item to process it later,
 * ensuring the interrupt handler remains fast.
 */
void us1800_midi_in_urb_complete(struct urb *urb);

/**
 * tascam_midi_out_urb_complete() - Completion handler for MIDI OUT bulk URB.
 * @urb: The completed URB.
 *
 * This function runs in interrupt context. It marks the output URB as no
 * longer in-flight. It then re-schedules the work handler to check for and
 * send any more data waiting in the ALSA buffer. This is a safe, non-blocking
 * way to continue the data transmission chain.
 */
void us1800_midi_out_urb_complete(struct urb *urb);

/**
 * tascam_create_midi() - Create and initialize the ALSA rawmidi device.
 * @tascam: The driver instance.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
int us1800_create_midi(struct us1800_card *tascam);

#include "us1800_pcm.h"

#endif /* __US1800_H */
