import Foundation
import Network

class UdpManager {
    private var listener: NWListener?
    private var connection: NWConnection?
    private let port: NWEndpoint.Port = 12345 // Change to your desired port

    init() {
        startListening()
    }

    private func startListening() {
        do {
            listener = try NWListener(using: .udp, on: port)
            listener?.newConnectionHandler = { [weak self] newConnection in
                self?.connection = newConnection
                self?.receive(on: newConnection)
                newConnection.start()
            }
            listener?.start(queue: .main)
            print("Listening on port \(port)")
        } catch {
            print("Failed to start listening: \(error)")
        }
    }

    private func receive(on connection: NWConnection) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 1024) { [weak self] data, context, isComplete, error in
            if let data = data, let message = String(data: data, encoding: .utf8) {
                print("Received message: \(message)")
                // Handle the received message
            }
            if let error = error {
                print("Receive error: \(error)")
                return
            }
            self?.receive(on: connection) // Continue receiving
        }
    }

    func send(message: String, to host: String) {
        guard let connection = connection else {
            print("No active connection")
            return
        }
        
        let data = message.data(using: .utf8)
        connection.send(content: data, completion: .contentProcessed { error in
            if let error = error {
                print("Send error: \(error)")
            } else {
                print("Sent message: \(message)")
            }
        })
    }

    func stop() {
        listener?.cancel()
        connection?.cancel()
        print("Stopped UDP manager")
    }
}