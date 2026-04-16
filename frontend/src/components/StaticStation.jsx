import React, { useEffect } from "react";
import { Slide, ToastContainer, toast } from 'react-toastify';

function StaticStation({ setStaticStation }) {
    // this cointain confidential info. Is only accessible from the company VPN, so it's safe to hardcode it here.
    // It is used to check if the user is connected to the company VPN before allowing access to certain API endpoints.
    const VPN_Addr = "10.6.0.3";

    useEffect(() => {
        fetch(`http://${VPN_Addr}:5000/api/static-station`)
        .then((response) => {
            if (response.ok) {
                toast.success('Dati stazioni fisse caricati con successo!');
                return response.json();
            } else {
                toast.error('Errore nel caricamento dei dati delle stazioni fisse. Assicurati di essere connesso alla VPN aziendale.');
                throw new Error('Errore nella richiesta: ' + response.status);
            }
        })
        .then((data) => {
            setStaticStation(data);
        })
        .catch((error) => {
            console.error('Errore durante il fetch:', error);
        });
    }, [setStaticStation]);

    return null;
}

export default StaticStation;