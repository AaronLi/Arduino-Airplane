function initMap() {
  var uluru = {lat: 42.24021040494719, lng: -82.98437118530273};
  var map = new google.maps.Map(document.getElementById('map'), {
    zoom: 16,
    center: uluru,
    clickableIcons: false,
    streetViewControl: false,
    mapTypeId: "hybrid",
    mapTypeControl: false
  });
  flightCoordinates = [
      uluru
    ];
  flightPath = new google.maps.Polyline({
    path: flightCoordinates,
    //geodesic: true,
    strokeColor: '#0000FF',
    strokeOpacity: 1.0,
    strokeWeight: 3,
  });
  flightPath.setMap(map);
  var marker = new google.maps.Marker({
    position: uluru,
    map: map
  });
  google.maps.event.addListener(map, 'click', function(e){
    console.log('Position:', e.latLng.toString());

    comSock.send(e.latLng.toString());
    var marker = new google.maps.Marker({
      position: e.latLng,
      map: map
    });
    var path = flightPath.getPath();
    path.push(e.latLng);
    console.log(path);
  })  
}
