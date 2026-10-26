// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2025 Šerif Rami <ramiserifpersia@gmail.com>
// Revised October 2026 Paul Taggart <pjtggrt@gmail.com>

#include "us1800.h"

/*
 * us1800_midi_in_work_handler() - Deferred work for processing MIDI input.
 * @work: The work_struct instance.
 *
 * This function runs in a thread context. It safely reads raw USB data from
 * the kfifo, processes it by stripping protocol-specific padding bytes, and
 * passes the clean MIDI data to the ALSA rawmidi subsystem.
 */
static void us1800_midi_in_work_handler(struct work_struct *work)
{
	struct us1800_card *us1800 =
		container_of(work, struct us1800_card, midi_in_work);
	u8 buf[9];
	u8 clean_buf[8];
	unsigned int count, clean_count;

	if (!us1800->midi_in_substream)
		return;

	while (kfifo_out_spinlocked(&us1800->midi_in_fifo, buf, sizeof(buf),
				    &us1800->midi_in_lock) == sizeof(buf)) {
		clean_count = 0;
		for (count = 0; count < 8; ++count) {
			if (buf[count] != 0xfd)
				clean_buf[clean_count++] = buf[count];
		}

		if (clean_count > 0)
			snd_rawmidi_receive(us1800->midi_in_substream,
					    clean_buf, clean_count);
	}
}

void us1800_midi_in_urb_complete(struct urb *urb)
{
	struct us1800_card *us1800 = urb->context;
	int ret;

	if (!us1800)
		goto out;

	if (urb->status) {
		if (urb->status != -ENOENT && urb->status != -ECONNRESET &&
		    urb->status != -ESHUTDOWN && urb->status != -EPROTO) {
			dev_err_ratelimited(us1800->card->dev,
					    "MIDI IN URB failed: status %d\n",
					    urb->status);
		}
		goto out;
	}

	if (atomic_read(&us1800->midi_in_active) &&
	    urb->actual_length > 0) {
		kfifo_in_spinlocked(&us1800->midi_in_fifo, urb->transfer_buffer,
				    urb->actual_length, &us1800->midi_in_lock);
		schedule_work(&us1800->midi_in_work);
	}

	usb_get_urb(urb);
	usb_anchor_urb(urb, &us1800->midi_in_anchor);
	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret < 0) {
		dev_err(us1800->card->dev,
			"Failed to resubmit MIDI IN URB: error %d\n", ret);
		usb_unanchor_urb(urb);
		goto out;
	}

out:
	usb_put_urb(urb);
}

/*
 * us1800_midi_in_open() - Opens the MIDI input substream.
 * @substream: The ALSA rawmidi substream to open.
 *
 * This function stores a reference to the MIDI input substream in the
 * driver's private data.
 *
 * Return: 0 on success.
 */
static int us1800_midi_in_open(struct snd_rawmidi_substream *substream)
{
	struct us1800_card *us1800 = substream->rmidi->private_data;

	us1800->midi_in_substream = substream;
	return 0;
}

/*
 * us1800_midi_in_close() - Closes the MIDI input substream.
 * @substream: The ALSA rawmidi substream to close.
 *
 * Return: 0 on success.
 */
static int us1800_midi_in_close(struct snd_rawmidi_substream *substream)
{
	return 0;
}

/*
 * us1800_midi_in_trigger() - Triggers MIDI input stream activity.
 * @substream: The ALSA rawmidi substream.
 * @up: Boolean indicating whether to start (1) or stop (0) the stream.
 *
 * This function starts or stops the MIDI input URBs based on the 'up'
 * parameter. When starting, it resets the kfifo and submits all MIDI input
 * URBs. When stopping, it kills all anchored MIDI input URBs and cancels the
 * associated workqueue.
 */
static void us1800_midi_in_trigger(struct snd_rawmidi_substream *substream,
				   int up)
{
	struct us1800_card *us1800 = substream->rmidi->private_data;
	int i, err;

	if (up) {
		if (atomic_xchg(&us1800->midi_in_active, 1) == 0) {
			if (!atomic_read(&us1800->playback_active))
				us1800_zero_trigger(us1800, true);
			scoped_guard(spinlock_irqsave, &us1800->midi_in_lock)
			{
				kfifo_reset(&us1800->midi_in_fifo);
			}

			for (i = 0; i < NUM_MIDI_IN_URBS; i++) {
				usb_get_urb(us1800->midi_in_urbs[i]);
				usb_anchor_urb(us1800->midi_in_urbs[i],
						       &us1800->midi_in_anchor);
				err = usb_submit_urb(us1800->midi_in_urbs[i],
							     GFP_KERNEL);
				if (err < 0) {
					dev_err(us1800->card->dev,
						"Failed to submit MIDI IN URB %d: %d\n",
						i, err);
					usb_unanchor_urb(
							us1800->midi_in_urbs[i]);
					usb_put_urb(us1800->midi_in_urbs[i]);
				}
			}
		}
	} else {
		if (atomic_xchg(&us1800->midi_in_active, 0) == 1) {
			usb_kill_anchored_urbs(&us1800->midi_in_anchor);
			cancel_work_sync(&us1800->midi_in_work);
		}
		if (atomic_read(&us1800->playback_zero))
			us1800_zero_trigger(us1800, false);
	}
}

/*
 * us1800_midi_in_ops - ALSA rawmidi operations for MIDI input.
 *
 * This structure defines the callback functions for MIDI input stream
 * operations, including open, close, and trigger.
 */
static const struct snd_rawmidi_ops us1800_midi_in_ops = {
	.open = us1800_midi_in_open,
	.close = us1800_midi_in_close,
	.trigger = us1800_midi_in_trigger,
};

void us1800_midi_out_urb_complete(struct urb *urb)
{
	struct us1800_card *us1800 = urb->context;
	int i, urb_index = -1;

	if (urb->status) {
		goto out;
	}

	if (!us1800)
		goto out;

	for (i = 0; i < NUM_MIDI_OUT_URBS; i++) {
		if (us1800->midi_out_urbs[i] == urb) {
			urb_index = i;
			break;
		}
	}

	if (urb_index < 0) {
		dev_err_ratelimited(us1800->card->dev,
				    "Unknown MIDI OUT URB completed!\n");
		goto out;
	}

	scoped_guard(spinlock_irqsave, &us1800->midi_out_lock)
	{
		clear_bit(urb_index, &us1800->midi_out_urbs_in_flight);
	}

	if (atomic_read(&us1800->midi_out_active))
		schedule_work(&us1800->midi_out_work);

out:
	usb_put_urb(urb);
}

/*
 * us1800_midi_out_work_handler() - Deferred work for sending MIDI data
 * @work: The work_struct instance.
 *
 * This function handles the proprietary output protocol: take the raw MIDI
 * message bytes from the application, place them at the start of a 9-byte
 * buffer, pad the rest with 0xFD, and add a terminator byte (0xe0).
 * This function pulls as many bytes as will fit into one packet from the
 * ALSA buffer and sends them.
 */
static void us1800_midi_out_work_handler(struct work_struct *work)
{
	struct us1800_card *us1800 =
		container_of(work, struct us1800_card, midi_out_work);
	struct snd_rawmidi_substream *substream = us1800->midi_out_substream;
	int i;

	if (!substream || !atomic_read(&us1800->midi_out_active))
		return;

	while (snd_rawmidi_transmit_peek(substream, (u8[]){ 0 }, 1) == 1) {
		int urb_index;
		struct urb *urb;
		u8 *buf;
		int bytes_to_send;

		scoped_guard(spinlock_irqsave, &us1800->midi_out_lock) {
			urb_index = -1;
			for (i = 0; i < NUM_MIDI_OUT_URBS; i++) {
				if (!test_bit(
					    i,
					    &us1800->midi_out_urbs_in_flight)) {
					urb_index = i;
					break;
				}
			}

			if (urb_index < 0)
				return; /* No free URBs, will be rescheduled by
					 * completion handler
					 */

			urb = us1800->midi_out_urbs[urb_index];
			buf = urb->transfer_buffer;
			bytes_to_send = snd_rawmidi_transmit(substream, buf, 8);

			if (bytes_to_send <= 0)
				break; /* No more data */

			if (bytes_to_send < 9)
				memset(buf + bytes_to_send, 0xfd,
				       9 - bytes_to_send);
			buf[8] = 0xe0;

			set_bit(urb_index, &us1800->midi_out_urbs_in_flight);
			urb->transfer_buffer_length = 9;
		}

		usb_get_urb(urb);
		usb_anchor_urb(urb, &us1800->midi_out_anchor);
		if (usb_submit_urb(urb, GFP_KERNEL) < 0) {
			dev_err_ratelimited(
					us1800->card->dev,
					"Failed to submit MIDI OUT URB %d\n",
										urb_index);
			scoped_guard(spinlock_irqsave, &us1800->midi_out_lock)
			{
				clear_bit(urb_index,
					  &us1800->midi_out_urbs_in_flight);
			}
			usb_unanchor_urb(urb);
			usb_put_urb(urb);
			break; /* Stop on error */
		}
	}
}

/*
 * us1800_midi_out_open() - Opens the MIDI output substream.
 * @substream: The ALSA rawmidi substream to open.
 *
 * This function stores a reference to the MIDI output substream in the
 * driver's private data and initializes the MIDI running status.
 *
 * Return: 0 on success.
 */
static int us1800_midi_out_open(struct snd_rawmidi_substream *substream)
{
	struct us1800_card *us1800 = substream->rmidi->private_data;

	us1800->midi_out_substream = substream;
	/* Initialize the running status state for the packet packer. */
	us1800->midi_running_status = 0;
	return 0;
}

/*
 * us1800_midi_out_close() - Closes the MIDI output substream.
 * @substream: The ALSA rawmidi substream to close.
 *
 * Return: 0 on success.
 */
static int us1800_midi_out_close(struct snd_rawmidi_substream *substream)
{
	return 0;
}

/*
 * us1800_midi_out_drain() - Drains the MIDI output stream.
 * @substream: The ALSA rawmidi substream.
 *
 * This function cancels any pending MIDI output work and kills all
 * anchored MIDI output URBs, ensuring all data is sent or discarded.
 */
static void us1800_midi_out_drain(struct snd_rawmidi_substream *substream)
{
	struct us1800_card *us1800 = substream->rmidi->private_data;
	bool in_flight = true;

	while (in_flight) {
		in_flight = false;
		for (int i = 0; i < NUM_MIDI_OUT_URBS; i++) {
			if (test_bit(i, &us1800->midi_out_urbs_in_flight)) {
				in_flight = true;
				break;
			}
		}
		if (in_flight)
			schedule_timeout_uninterruptible(1);
	}

	cancel_work_sync(&us1800->midi_out_work);
	usb_kill_anchored_urbs(&us1800->midi_out_anchor);
}

/*
 * us1800_midi_out_trigger() - Triggers MIDI output stream activity.
 * @substream: The ALSA rawmidi substream.
 * @up: Boolean indicating whether to start (1) or stop (0) the stream.
 *
 * This function starts or stops the MIDI output workqueue based on the
 * 'up' parameter.
 */
static void us1800_midi_out_trigger(struct snd_rawmidi_substream *substream,
				    int up)
{
	struct us1800_card *us1800 = substream->rmidi->private_data;

	if (up) {
		if (!atomic_read(&us1800->playback_active))
			us1800_zero_trigger(us1800, true);
		atomic_set(&us1800->midi_out_active, 1);
		schedule_work(&us1800->midi_out_work);
	} else {
		atomic_set(&us1800->midi_out_active, 0);
		if (atomic_read(&us1800->playback_zero))
			us1800_zero_trigger(us1800, false);
	}
}

/*
 * us1800_midi_out_ops - ALSA rawmidi operations for MIDI output.
 *
 * This structure defines the callback functions for MIDI output stream
 * operations, including open, close, trigger, and drain.
 */
static const struct snd_rawmidi_ops us1800_midi_out_ops = {
	.open = us1800_midi_out_open,
	.close = us1800_midi_out_close,
	.trigger = us1800_midi_out_trigger,
	.drain = us1800_midi_out_drain,
};

int us1800_create_midi(struct us1800_card *us1800)
{
	int err;

	err = snd_rawmidi_new(us1800->card, "US-1800 MIDI", 0, 1, 1,
			      &us1800->rmidi);
	if (err < 0)
		return err;

	strscpy(us1800->rmidi->name, "US1800 MIDI",
		sizeof(us1800->rmidi->name));
	us1800->rmidi->private_data = us1800;

	snd_rawmidi_set_ops(us1800->rmidi, SNDRV_RAWMIDI_STREAM_INPUT,
			    &us1800_midi_in_ops);
	snd_rawmidi_set_ops(us1800->rmidi, SNDRV_RAWMIDI_STREAM_OUTPUT,
			    &us1800_midi_out_ops);

	us1800->rmidi->info_flags |= SNDRV_RAWMIDI_INFO_INPUT |
				     SNDRV_RAWMIDI_INFO_OUTPUT |
				     SNDRV_RAWMIDI_INFO_DUPLEX;

	INIT_WORK(&us1800->midi_in_work, us1800_midi_in_work_handler);
	INIT_WORK(&us1800->midi_out_work, us1800_midi_out_work_handler);

	return 0;
}
