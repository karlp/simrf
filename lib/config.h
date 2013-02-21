/*
 * Configuration of the library.
 */

/*
 * This determines whether 2 bytes of padding are inserted after the 802.15.4
 * header.  This is required (set to 1) if you are sending to an xbee series
 * one, in MAC Mode 0 or 3. In a "proper" 802.15.4 network, this should be
 * set to 0.
 */
#define PAD_DIGI_HEADER 0
