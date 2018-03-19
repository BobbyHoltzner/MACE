const width = window.screen.width;
// const height = window.screen.height;

export const styles = {
    aircraftCommsContainer: {
        // backgroundColor: colors.orange700,
        position: 'absolute' as 'absolute',
        bottom: 0,
        left: 50 + '%',
        // height: 64,
        zIndex: 9999,
        width: 50 + "%",
        display: "flex",
        justifyContent: "center" as "center",
        alignItems: "center" as "center",
        marginLeft: -width*0.25
    },
    buttonStyle: {
        margin: 5
    }
}