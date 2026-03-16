NimBLE Host ATT Client Reference
--------------------------------

Introduction
~~~~~~~~~~~~

The Attribute Protocol (ATT) is a mid-level protocol that all BLE devices use to exchange data. Data is exchanged when
an ATT client reads or writes an attribute belonging to an ATT server. Any device that needs to send or receive data
must support both the client and server functionality of the ATT protocol. The only devices which do not support ATT
are the most basic ones: broadcasters and observers (i.e., beaconing devices and listening devices).

Most ATT functionality is not interesting to an application. Rather than use ATT directly, an application uses the
higher level GATT profile, which sits directly above ATT in the host. NimBLE exposes the few bits of ATT functionality
which are not encompassed by higher level GATT functions. This section documents the ATT functionality that the NimBLE
host exposes to the application.

API
~~~~~~

.. doxygengroup:: bt_host
    :content-only:
    :members:

NimBLE Host Enhanced ATT (EATT) Reference
---------------------------------------

Introduction
~~~~~~~~~~~~

The Enhanced Attribute Protocol (EATT) is an improved version of the Attribute Protocol (ATT) introduced in Bluetooth 5.2. It allows for concurrent ATT transactions, decreased latency, and improved reliability by running over L2CAP Enhanced Credit-Based Flow Control (ECFC) channels.

Unlike the legacy ATT, which runs on a fixed channel (CID 4) and is sequential (one transaction at a time), EATT can use multiple dynamic L2CAP channels. This enables the stack to perform parallel GATT operations, such as reading multiple characteristics simultaneously or handling notifications/indications while other operations are pending.

EATT is largely handled transparently by the NimBLE host. When enabled, the stack will automatically attempt to establish EATT bearers with peer devices that support it. Once established, upper layers (GATT Client/Server) will utilize these channels for ATT traffic.

Configuration
~~~~~~~~~~~~~

To use EATT, it must be enabled in the system configuration and properly configured during host initialization.

System Configuration (syscfg)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The number of EATT channels is controlled by the ``BLE_EATT_CHAN_NUM`` setting. This must be set to a value greater than 0 to enable EATT support in the build.

.. code-block:: yaml

    syscfg.vals:
        BLE_EATT_CHAN_NUM: 5  # Example: Support up to 5 EATT channels

Host Configuration (ble_hs_cfg)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

At runtime, the number of EATT channels to be established per connection is configured via the ``ble_hs_cfg.eatt`` field. This should be set before starting the host.

.. code-block:: c

    void app_on_sync(void) {
        /* ... */
    }

    int main(void) {
        /* ... sysinit ... */
        
        /* Configure maximum number of EATT channels per connection */
        ble_hs_cfg.eatt = 5;

        /* ... ble_hs_start ... */
    }

Usage
~~~~~

Automatic Establishment
^^^^^^^^^^^^^^^^^^^^^^^

EATT channels are L2CAP Connection Oriented Channels (CoC) and require an encrypted link. The NimBLE host automatically manages the establishment of these channels:

1.  **Initialization**: `ble_eatt_init` is called automatically during stack initialization if `BLE_EATT_CHAN_NUM > 0`. This registers a GAP event listener and creates an L2CAP server for the EATT Protocol Service Multiplexer (PSM).
2.  **Encryption**: When a connection is encrypted (`BLE_GAP_EVENT_ENC_CHANGE`), the internal `ble_eatt_gap_event` handler is triggered.
3.  **Connection**: If the device is the Central (Master), it initiates the creation of EATT channels up to the configured `ble_hs_cfg.eatt` limit. If it is the Peripheral (Slave), it accepts incoming connection requests.

Transparent Integration
^^^^^^^^^^^^^^^^^^^^^^^

The GATT layer is aware of EATT and utilizes it when available:

-   **Transmission**: The `ble_att_tx` function checks if EATT is enabled. If a specific EATT Channel ID (CID) is provided, it delegates to `ble_eatt_tx` to send the packet over the L2CAP ECFC channel.
-   **Channel Selection**: Functions like `ble_eatt_get_available_chan_cid` are used by the GATT client to find an idle EATT channel for a new transaction.

API Reference
~~~~~~~~~~~~~

The following functions are the core of the EATT implementation. While many are internal to the stack, understanding them clarifies how EATT operates.

.. note::
   Most applications do not need to call these functions directly. EATT is handled automatically by the stack when configured.

**ble_eatt_init**

.. code-block:: c

   int ble_eatt_init(ble_eatt_att_rx_fn att_rx_cb)

Initializes the EATT module. It allocates memory for channels, registers the GAP event listener for encryption changes, and registers the L2CAP server for incoming EATT connections.

**ble_eatt_tx**

.. code-block:: c

   int ble_eatt_tx(uint16_t conn_handle, uint16_t cid, struct os_mbuf *txom)

Transmits an ATT packet over a specific EATT channel (identified by `cid`). This handles the L2CAP segmentation and credit-based flow control.

**ble_eatt_get_available_chan_cid**

.. code-block:: c

   uint16_t ble_eatt_get_available_chan_cid(uint16_t conn_handle, uint8_t op)

Finds an available (idle) EATT channel for the given connection. If an EATT channel is available, it marks it as busy with the specified operation (`op`) and returns its CID. If no EATT channel is available, it returns the legacy ATT CID or 0.

**ble_eatt_release_chan**

.. code-block:: c

   void ble_eatt_release_chan(uint16_t conn_handle, uint8_t op)

Releases a previously claimed EATT channel, marking it as idle and available for new transactions.
