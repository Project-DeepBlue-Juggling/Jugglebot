---
id: IDEA-19
kind: idea
title: Improve "Next Sitting" Runsheet
status: inbox
created: '2026-11-08'
updated: '2026-11-08T10:48:41Z'
owners: []
priority: medium
effort: m
hardware: none
blocked_by: []
links: []
filed_by: harrison
---

Currently, having all Items and their associated steps is nice, but not nearly as convenient as it could be. 

I would much prefer to have a single cohesive runsheet that integrates all the Items' steps, in logical sequence.

This will likely reward the construction of a custom Claude slash-command that I invoke just before beginning a Sitting. I should be able to say what kind of Sitting it will be (robot [which includes BB by default], bench leg, bench hand) and the command will have Claude order all the steps for the Items that need Sitting time.

It would be nice to have lower-priority Items' steps indicated as 'optional', with a link to the Item they're for, so I can evaluate whether I want to run the step or not.

To make my goal clear: when I am ready for a Sitting, I want a single runsheet that I can step through, instructing me what commands to run, when a new ROS2 session should be initiated, etc (please come back to me with your suggestions for what to include on each step and we can agree on this before building anything).

Each step must be clear, and anything that must happen before a given step must be placed before it in the sheet. It should NOT be assumed that I will read the whole runsheet before beginning the Sitting.

The buildout of this idea will reward our discussion of its intricacies. Please come back to me with any questions you have before we build this.

Filed from the Robot tab at 2026-11-08T21:48:40+11:00.
