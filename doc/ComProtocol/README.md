# ComProtocol

## Architecture

Server - Client Architecture
Just One Client (?)
Send message, wait for response or timeout, then move on.
Fixed Frame Length = 11 Bytes
Parsing "automatically" using "raw" field.

---

## CTRL (Channel 0)

DLC >= 1
D0: Multiplexer

### SYNC (D0 = 0)

DLC = 5
Server sends SYNC package with a timestamp.
SYNC sent every 100ms.
Client responds with ACK and the same timestamp.
Server can then calculate round-trip time and use it as timeout.
Possible to do an average timeout from several SYNC packages.

#### Server

D1-D4 = timestamp uint32_t

#### Client

D1-D4 = timestamp uint32_t
Returns same timestamp but on D0 = 1 (ACK)

---

### ACK (D0 = 1)

DLC = 1 (Except for SYNC)
Acknowledgement for any message

---

### NACK (D0 = 2)

DLC = 1
Not Acknowledgement for any message.
Last message should be resent.

---

## Timeout

Wait-time between message sent, and response received.
Defaults to 100ms (?)
Calculated at start of communication with SYNC.

---

## Priorities

No priority system.
FIFO Queue

---

## Checksum

Add fields (channel, dlc, data). Checksum done with modulo 255 ( checksum = sum % UINT8_MAX ).
