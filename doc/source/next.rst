.. _next:

Next steps
==========

The following services need to be implemented:

  - ``/naoqi/set_life_status`` to turn the life behavior on or off
  - ``/naoqi/start_asr`` and ``/naoqi/stop_asr``
  - ``/naoqi/speak`` (TODO: look at audio common)

The following items need love:

  - info needs to be a proper struct, not fake JSON
  - bumpers are not implemented
  - generic audio
  - octomap
  - LEDs

To get things faster:

  - check how DCM access is done with B-Human: https://github.com/bhuman/BHumanCodeRelease/blob/master/Src/libbhuman/bhuman.cpp#L768


Go back to the :ref:`index <main menu>`.