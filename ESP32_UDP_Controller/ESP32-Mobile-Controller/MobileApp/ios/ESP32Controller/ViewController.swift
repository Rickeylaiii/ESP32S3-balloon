import UIKit

class ViewController: UIViewController {
    
    let udpManager = UdpManager()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Set up the UDP manager
        udpManager.startReceiving()
        
        // Additional UI setup can be done here
    }
    
    @IBAction func sendCommand(_ sender: UIButton) {
        // Example command to send to ESP32
        let command = "Hello ESP32"
        udpManager.sendMessage(command)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        udpManager.stopReceiving()
    }
}