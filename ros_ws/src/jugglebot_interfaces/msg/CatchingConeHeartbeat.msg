# For relaying the heartbeat being received from the Catching Cone

bool connected               # True if a CAN heartbeat was received within the timeout window
uint8 state                  # Cone state (0=BOOT, 1=UNSYNCED, 2=READY, 127=ERROR)
uint8 state_data             # Sync-jitter RMS {us}, or error code when state == ERROR
uint8 sync_rms_us            # Time-sync jitter RMS {us} (0 when in ERROR)
uint8 last_catch_seq         # Sequence number of the most recent catch
uint32 ms_since_last_catch   # Time since the last catch {ms}
bool time_synced             # True if the cone is locked to the global clock
bool have_any_catch          # True once the cone has seen at least one catch
                             # (else last_catch_seq and ms_since_last_catch are placeholders)
