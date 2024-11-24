// This is electric-load main program.
// License: MIT
// Author: 2024, Hiroshi Nakajima

use log::*;
use std::{thread, sync::Arc, sync::Mutex};
use esp_idf_hal::task;
use std::io::Error;
use std::time::Duration;
use embedded_svc::http::client::Client;
use embedded_svc::http::Method;
use esp_idf_svc::http::client::{EspHttpConnection, Configuration};

use anyhow::Result;
use crate::CurrentLog;

struct TransferData {
    body: String,
    txreq: bool,
}

#[derive(Clone)]
pub struct ServerInfo {
    pub server: String,
    pub influxdb_measurement: String,
    pub influxdb_api_key: String,
    pub influxdb_api: String,
    pub influxdb_tag: String,
}

impl ServerInfo {
    pub fn new(server: String, api_key: String, api: String, measurement: String, tag: String) -> Self {
        ServerInfo {
            server: server,
            influxdb_measurement: measurement,
            influxdb_api_key: api_key,
            influxdb_api: api,
            influxdb_tag: tag,
        }
    }
}

pub struct Transfer {
    data: Arc<Mutex<TransferData>>,
    server: ServerInfo,
}

impl Transfer {
    pub fn new(server: ServerInfo) -> Self {
        Transfer { data: Arc::new(Mutex::new(
            TransferData { body: "".to_string(), txreq: false })),
            server: server}
    }

    pub fn start(&mut self) -> Result<(), Error>
    {
        let data = self.data.clone();
        let server_info = self.server.clone();
        let _th = thread::spawn(move || -> anyhow::Result<()> {
            info!("Start transfer thread.");    

            loop {
                task::wait_notification(100);
                let http = EspHttpConnection::new(
                    &Configuration {
                        use_global_ca_store: true,
                        crt_bundle_attach: Some(esp_idf_svc::sys::esp_crt_bundle_attach),
                        timeout: Some(Duration::from_secs(10 as u64)),
                        ..Default::default()
                    })?;
    
                let mut client = Client::wrap(http);
    
                let mut lck = data.lock().unwrap();
                if lck.txreq == false {
                    drop(lck);
                    continue;
                }
                let request = format!("{}", lck.body);
                drop(lck);
                // info!("Transfer data: {}", request);                
                let ret = Self::transfer(&mut client, &server_info, request);
                lck = data.lock().unwrap();
                match ret {
                    Ok(()) => { lck.txreq = false; },
                    Err(e) => { info!("{}", e) },
                }
                lck.body.clear();
                drop(lck);
            }
        });

        Ok(())
    }

    fn transfer(client: &mut Client<EspHttpConnection>, server_info: &ServerInfo, body_data: String) -> anyhow::Result<()>
    {
        let authorization = &format!("Token {}", server_info.influxdb_api_key);
        let headers : [(&str, &str); 2] = [
                ("Authorization", authorization),
                ("Content-Type", "application/json"),
            ];
        let url = format!("http://{}{}", server_info.server, server_info.influxdb_api);
        // info!("URL: {}", url);
        let mut request = client.request(Method::Post, 
               url.as_str(),
                &headers)?;
        let body = body_data.as_bytes();
        request.write(body)?;
        let mut response = request.submit()?;
        let res_status = response.status();
        // info!("Response status: {:?}", res_status);
        match res_status {
            204 => {
                return Ok(());
            },
            _ => {
                let mut response_buf = [0u8; 4096];
                response.read(&mut response_buf)?;
                let res_str = std::str::from_utf8(&response_buf).unwrap_or("<invalid UTF-8>");        
                info!("Response: {}", res_str);
                return Err(anyhow::anyhow!("Failed to transfer data."));
            }
        }
    }


    pub fn set_transfer_data(&mut self, data: &Vec<CurrentLog>) -> usize
    {
        if data.len() == 0 {
            return 0;
        }
        let mut lck = self.data.lock().unwrap();
        if lck.txreq == true {
            // info!("Transfer request is already pending.");
            return 0;
        }
        let mut count = 0;
        for it in data {
            lck.body.push_str(
                &format!("{},tag={} current={:.5},voltage={:.5},power={:.5},bat={:.2},temp={:.1},rpm={},pwm={} {}\n",
                    self.server.influxdb_measurement,
                    self.server.influxdb_tag,
                    it.current,
                    it.voltage,
                    it.power,
                    it.battery,
                    it.temp,
                    it.rpm,
                    it.pwm,
                    it.clock,
            ));
            count += 1;
            if count == 128 {
                info!("Chunk data");
                break;
            }
        }
        lck.txreq = true;
        count as usize
    }
}
