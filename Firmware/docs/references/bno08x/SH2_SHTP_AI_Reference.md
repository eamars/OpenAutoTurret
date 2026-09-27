---
title: "SH-2 SHTP - AI/Grep-Friendly Channel/Advertisement Reference"
software: "SH-2"
document_source: "SH-2-SHTP-Reference-Manual.pdf"
source_document_number: "1000-3600"
source_revision: "1.6"
source_date_printed: "2017"
copyright_page: "2019"
source_pages: 8
document_type: "normalized supplement + source-preserving transcript"
grep_keywords:
  - SHTP
  - SH-2
  - channel
  - advertisement
  - inputNormal
  - inputWake
  - inputGyroRv
  - control
  - device
  - ReportLengths
---

# SH-2 SHTP - AI/Grep-Friendly Reference

This file converts **SH-2 SHTP Reference Manual 1000-3600 revision 1.6** into a concise grep-friendly form and appends the source text.

## 0. Scope warning

`[SOURCE]`

This document is **not** the full Sensor Hub Transport Protocol specification.

It is an **SH-2-specific supplement** describing:

- SH-2 use of SHTP;
- SH-2 channel roles;
- SH-2 advertisement tags.

The full generic SHTP protocol is referenced as:

```text
1000-3535 Sensor Hub Transport Protocol Reference Manual
```

but that document is not part of the uploaded set.

---

# 1. Channel model

The document expresses channel numbers relative to a base `N`.

| Channel | Name | Host write | Host read |
|---:|---|---|---|
| `N` | `device` | one-byte executable command | one-byte executable response |
| `N+1` | `control` | SH-2 reports/configuration commands | non-sensor configuration/control responses |
| `N+2` | `inputNormal` | unused | normal input reports |
| `N+3` | `inputWake` | unused | wake-sensor input reports |
| `N+4` | `inputGyroRv` | unused | Gyro Rotation Vector data only |

`N` can be selected at compile time or runtime depending on SH-2 implementation.

For BNO08X, the device datasheet resolves these to fixed channels:

```text
N   = 1 executable/device
N+1 = 2 control
N+2 = 3 inputNormal
N+3 = 4 inputWake
N+4 = 5 inputGyroRv
```

with SHTP channel `0` reserved for SHTP's own command channel.

---

# 2. Device channel

Host -> device, one byte per cargo:

```text
0 = reserved
1 = reset
2 = on
3 = sleep
4..255 = reserved
```

Device -> host:

```text
0 = reserved
1 = reset complete
2..255 = reserved
```

Multiple commands/responses in one device cargo are not supported.

---

# 3. Control channel

Host -> SH-2:

- reports sent to the hub;
- Set Feature;
- Get Feature;
- Force Flush;
- other control/configuration reports.

Normal/wakeup input sensor data is not sent here.

The newer SH-2 Reference Manual v1.9 should be used for current report IDs and exact report layouts.

---

# 4. inputNormal

Read-only from host perspective.

Cargo can contain one or more normal sensor input reports.

Each report begins with a Report ID and has a fixed length for the advertised firmware/report version.

---

# 5. inputWake

Read-only from host perspective.

Cargo contains wake-sensor input reports.

This separates wake traffic from normal/always-on traffic.

---

# 6. inputGyroRv

Read-only from host perspective.

Cargo contains only compact Gyro + Rotation Vector reports.

Important rules:

```text
one gyroRV report per SHTP transfer
never multiple gyroRV reports in one cargo
```

The SH-2 v1.9 reference further clarifies that Gyro-Integrated Rotation Vector report payloads omit the report ID because this channel is dedicated to that single report type.

---

# 7. Channel advertisement

The SH-2 advertises the following logical tags:

| Tag | Name | Value |
|---:|---|---|
| `1` | GUID | `1` |
| `8` | AppName | `executable` |
| `6` | NormalChannel | `N` |
| `9` | ChannelName | `device` |
| `1` | GUID | `2` |
| `8` | AppName | `sensorhub` |
| `6` | NormalChannel | `N+1` |
| `9` | ChannelName | `control` |
| `6` | NormalChannel | `N+2` |
| `9` | ChannelName | `inputNormal` |
| `7` | WakeChannel | `N+3` |
| `9` | ChannelName | `inputWake` |
| `6` | NormalChannel | `N+4` |
| `9` | ChannelName | `inputGyroRv` |
| `0x80` | Version | implementation dependent |
| `0x81` | Report Lengths | `(report ID, length)` pairs |

---

# 8. ReportLengths

`ReportLengths` exists to support compatibility between host and hub versions.

Representation:

```text
array of uint8 pairs:
(report_id, report_length)
```

The length includes:

```text
report ID byte + report contents
```

For cargo containing multiple reports:

```text
first byte = report ID
lookup length for that report ID
advance exactly that many bytes
repeat
```

`[CAUTION]` The dedicated gyroRV channel is a special case in later SH-2 documentation because its compact report omits a Report ID.

---

# 9. Timestamping

The supplement explicitly states that SH-2 uses the SHTP timestamping feature.

For the full SH-2 interpretation, see:

```text
Base Timestamp Reference 0xFB
Timestamp Rebase 0xFA
14-bit per-report delay in 100 us ticks
```

in `SH2_AI_Reference.md`.

---

# 10. CROSS_DOCUMENT_CONFLICTS

## Legacy Get Feature Response ID

`[CONFLICT]`

This older SH-2 SHTP supplement says in the control-channel description that a Get Feature Response is:

```text
report ID = 0xF2 followed by feature report ID
```

The **newer SH-2 Reference Manual v1.9** explicitly defines:

```text
0xF2 = Command Request
0xFC = Get Feature Response
```

The BNO08X datasheet v1.17 also uses `0xFC` for Get Feature Response.

**Implementation rule for this bundle: use `0xFC` for Get Feature Response. Treat the `0xF2` wording in this older supplement as legacy/outdated.**

---

# 11. Recommended parser architecture

```text
transport layer
  -> parse SHTP framing/header (generic SHTP spec 1000-3535 when available)

channel dispatcher
  channel 1 / device
  channel 2 / control
  channel 3 / inputNormal
  channel 4 / inputWake
  channel 5 / inputGyroRv

report parser
  -> use ReportLengths or known report definitions
  -> special-case gyroRV payload
  -> track sequence numbers independently
```

Do not make the sensor-report parser infer report type from payload shape when a report ID/channel already identifies it.

---

# 12. Suggested grep commands

```bash
grep -ni 'inputGyroRv' SH2_SHTP_AI_Reference.md
grep -ni 'ReportLengths' SH2_SHTP_AI_Reference.md
grep -ni 'CROSS_DOCUMENT_CONFLICTS' SH2_SHTP_AI_Reference.md
grep -ni '0xFC' SH2_SHTP_AI_Reference.md
grep -ni 'channel' SH2_SHTP_AI_Reference.md
```

---


# Source-preserving transcript

This appendix preserves the PDF's extracted text page-by-page for exhaustive grep/search. The normalized sections above should be preferred for implementation, while this transcript is useful for locating source wording, tables, figures, and fields that were not promoted into the normalized reference.


## PDF page 1

```text
      SH-2 SHTP Reference Manual


      Document Number: 1000-3600
      Document Revision: 1.6
                  Date: 2017




CEVA Technologies, Inc. 15245 Shady Grove Road, Suite 400 Rockville, MD 20850
                    © 2019, CEVA, Inc. All rights reserved
```


## PDF page 2

```text
                                                Table of Contents
LIST OF FIGURES ................................................................................................................. 2
1.0   INTRODUCTION ........................................................................................................ 3
   1.1       Intended Audience ................................................................................................................ 3
   1.2       Scope .................................................................................................................................... 3
   1.3       Revision History .................................................................................................................... 3

2.0       SH-2’S USES OF SHTP ............................................................................................. 4
   2.1       Host Interrupt and Timestamps ............................................................................................. 4
   2.2       SH-2 Channel Usage ............................................................................................................ 4
   2.3       Channel Advertisement ......................................................................................................... 5

3.0       REFERENCES ........................................................................................................... 6
4.0       NOTICES .................................................................................................................... 7




                                            © 2019 CEVA, Inc. All rights reserved.                                                                     1
```


## PDF page 3

```text
1000-3600                                                                                 SH-2 SHTP Reference Manual



                                               List of Figures
Figure 1: Document History ................................................................................................ 3
Figure 2: SH-2 Write Channel Usage ................................................................................. 4
Figure 3: SH-2 Advertisement ............................................................................................. 5




                                           © 2019 CEVA, Inc. All rights reserved.                                          2
```


## PDF page 4

```text
1000-3600                                                                SH-2 SHTP Reference Manual



1.0         Introduction
The SH-2 has two high level variants – the chip executable and the library. Both variants of the
SH-2 use the Sensor Hub Transport Protocol (SHTP) to communicate with the host. See [1] for
details of SHTP. This document supplements the SH-2 Reference Manual [2] with details about
the SHTP advertisement response and channel usage.

1.1      Intended Audience
This document is intended for application developers implementing products that use the SH-2.

1.2      Scope
This document describes how the SH-2 uses the SHTP.

1.3      Revision History
       Revision          Date                                  Description
 1.6
 1.5               02/16/2017         Deleted unused reference.
 1.4               12/14/2016         Added legal notices.
 1.3               11/17/2016         Defined version tag format. Add Gyro-Rotation vector channel.
 1.2               07/13/2015         Add force-flush command/response.
 1.1               03/30/2015         Minor updates following implementation.
 1.0               02/25/2015         Initial issue
                                  Figure 1: Document History




                                © 2019 CEVA, Inc. All rights reserved.                                3
```


## PDF page 5

```text
1000-3600                                                                          SH-2 SHTP Reference Manual



2.0         SH-2’s Uses of SHTP
2.1    Host Interrupt and Timestamps
The SH-2 uses the SHTP timestamping feature.

2.2    SH-2 Channel Usage
The SH-2 cargoes consist of predefined messages identified by report ID’s. See [2] for details of
these messages. To make message handling and identification easier the SH-2 uses certain
channels for specific purposes. The SH-2 channel usage is shown in Figure 2. The device
channel is used only on the chip executable variants.
   Channel                                         Use                                      Name        Direction
              Device commands. The commands are one byte. The cargo for a
              command is one byte. Sending multiple commands in one cargo is
              not supported. The defined commands are:
              0 – reserved
                                                                                                          Write
              1 – reset
              2 – on
              3 – sleep
      N                                                                                     device
              4-255 – reserved
              Device responses. The responses are one byte. The cargo for a
              response is one byte. Sending multiple responses in one cargo is not
              supported. The defined responses are:
                                                                                                          Read
              0 – reserved
              1 – reset complete
              2-255 – reserved
              All reports sent to the hub. Each report is identified by its report ID.
              Report lengths are fixed. Set feature reports are defined as report ID
              = 0xFD followed by the feature report. The full feature report,
              including its report ID as defined in [2], is sent. Get feature reports                     Write
              are defined as: report ID = 0xFE followed by the ID of the feature
              report to get. Force-flush reports are defined as: reportID=0xF0
      N+1     followed by the ID of the sensor to be flushed.                               control
              All reports other than sensor data reports. Each report is identified by
              a report ID. Report lengths are fixed. Get feature responses are
              defined as: report ID = 0xF2 followed by the ID of the feature report                       Read
              to get.
              All other reports are sent as they are defined in [2].
              Unused                                                                                      Write
      N+2     Input reports. The cargo consists of one or more input reports. Each        inputNormal
                                                                                                          Read
              report is identified by its report ID. Report lengths are fixed.
              Unused                                                                                      Write
      N+3     Input reports for wake sensors. The cargo consists of one or more           inputWake
              input reports. Each report is identified by its report ID. Report lengths                   Read
              are fixed.
              Unused                                                                                      Write
              Gyro Rotation Vector data. The cargo consists exclusively of
      N+4     compact Gyro + Rotation Vector reports. See [2] for report format.          inputGyroRv
                                                                                                          Read
              Gyro + RV reports are sent one at a time: a single SHTP transfer will
              never contain multiple Gyro + RV reports.
                                    Figure 2: SH-2 Write Channel Usage




                                      © 2019 CEVA, Inc. All rights reserved.                                      4
```


## PDF page 6

```text
1000-3600                                                                  SH-2 SHTP Reference Manual



2.3    Channel Advertisement
The SH-2 advertises the values listed in Figure 3.
                        Tag            Tag Name                     Value
                         1     GUID                        1
                         8     AppName                     executable
                         6     NormalChannel               N
                         9     ChannelName                 device
                         1     GUID                        2
                         8     AppName                     sensorhub
                         6     NormalChannel               N+1
                         9     ChannelName                 control
                         6     NormalChannel               N+2
                         8     ChannelName                 inputNormal
                         7     WakeChannel                 N+3
                         9     ChannelName                 inputWake
                         6     NormalChannel               N+4
                         9     ChannelName                 inputGyroRv
                        0x80   Version                     Varies
                        0x81   Report Lengths              Array of (report ID,
                                                           length) pairs
                                   Figure 3: SH-2 Advertisement
The value for N may be selected at compile time or run time depending on the specific
implementation of SH-2.
The version tag uses the same format as the version tag used by SHTP [1].
The ReportLengths tag is provided to support compatibility between different versions of host
and sensor hub. The array provided consists of pairs of unsigned 8 bit integers, in (report ID,
length) order. Cargoes received from the hub may consist of multiple reports. The first byte will
always be a report ID, and the next report will start a fixed number of bytes after that (the length
indicated by the corresponding entry in ReportLengths). The length of a report always includes
the report ID itself, as well as the report contents.




                                 © 2019 CEVA, Inc. All rights reserved.                            5
```


## PDF page 7

```text
1000-3600                                                              SH-2 SHTP Reference Manual



3.0         References
1. Hillcrest Laboratories, 1000-3535 Sensor Hub Transport Protocol Reference Manual.
2. Hillcrest Laboratories, 1000-3625 SH-2 Reference Manual.




                              © 2019 CEVA, Inc. All rights reserved.                           6
```


## PDF page 8

```text
1000-3600                                                                 SH-2 SHTP Reference Manual



4.0         Notices
© Copyright 08/2019 CEVA, Inc. and/or its subsidiaries (“CEVA”) All rights reserved. All
specifications are subject to change without notice.
Disclaimer: The information furnished herein is believed to be accurate and reliable. However,
the information is provided “AS IS”, without any express or implied warranty of any kind
including warranties of merchantability, non-infringement of intellectual property, or fitness for
any particular purpose.
In no event shall CEVA or its suppliers be liable for any claims and/or damages whatsoever
arising out of the use of or inability to use the materials. CEVA and its suppliers further do not
warrant the accuracy or completeness of the information, text, graphics or other items contained
within these materials. CEVA may make changes to these materials, or to the products
described within.


                                        www.ceva-dsp.com




                                               FOR MORE
                                             INFORMATION:




                                 © 2019 CEVA, Inc. All rights reserved.                           7
```
