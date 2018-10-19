picam360-server
****************

Introduction
============

This is capture software of picam360 camera.

.. _`picam360 Home Page`: https://www.picam360.com

picam360-server is maintained and supported by `OHMYDIGIFAB`_ and developed in
collaboration with a productive community of contributors.

.. _`OHMYDIGIFAB`: http://www.ohmydigifab.com/

License
=======

picam360-server is distributed under the GPL License.
See `Copyright.txt`_ for details.

.. _`Copyright.txt`: Copyright.txt

Building picam360-server
==============

Supported Platforms
-------------------

* Linux
* Apple macOS

Building picam360-server from Scratch
---------------------------

UNIX/Mac OSX/
^^^^^^^^^^^^^

You need to have a C++ compiler (supporting C++11) and a ``cmake``, a ``make`` installed.
Run the ``cmake /path/to/CMakeList.txt`` in the source directory of picam360-server.
You can use the ``--list`` option to see the supported options.
You may use the ``--prefix=<install_prefix>`` option to specify a custom
installation directory for picam360-server. You can run the ``cmake /path/to/CMakeList.txt`` from
within the picam360-server source directory or any other build directory of your
choice. Once this has finished successfully, run ``make`` and
``make install``.  In summary::

 $ cmake . && make && sudo make install
