function initMap() {
  var uluru = {lat: 42.24021040494719, lng: -82.98437118530273};
  var map = new google.maps.Map(document.getElementById('map'), {
    zoom: 18,
    center: uluru,
    clickableIcons: false,
    streetViewControl: false,
    mapTypeId: "hybrid",
    mapTypeControl: false
  });
  flightCoordinates = [
    ];
  flightPath = new google.maps.Polyline({
    path: flightCoordinates,
    //geodesic: true,
    strokeColor: '#0000FF',
    strokeOpacity: 1.0,
    strokeWeight: 3,
  });

  //---device location
  infoWindow = new google.maps.InfoWindow;
  // Try HTML5 geolocation.
  if (navigator.geolocation) {
    navigator.geolocation.getCurrentPosition(function(position) {
      var pos = {
        lat: position.coords.latitude,
        lng: position.coords.longitude
      };

      infoWindow.setPosition(pos);
      infoWindow.open(map);
      map.setCenter(pos);
    }, function() {
      handleLocationError(true, infoWindow, map.getCenter());
    });
  } else {
    // Browser doesn't support Geolocation
    handleLocationError(false, infoWindow, map.getCenter());
  }
  function handleLocationError(browserHasGeolocation, infoWindow, pos) {
    infoWindow.setPosition(pos);
    infoWindow.setContent(browserHasGeolocation ?
      'Error: The Geolocation service failed.' :
      'Error: Your browser doesn\'t support geolocation.');
    infoWindow.open(map);
  }
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
